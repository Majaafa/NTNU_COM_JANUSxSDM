//*************************************************************************
// JANUS is a simple, robust, open standard signalling method for         *
// underwater communications. See <http://www.januswiki.org> for details. *
//*************************************************************************
// Example software implementations provided by STO CMRE are subject to   *
// Copyright (C) 2008-2018 STO Centre for Maritime Research and           *
// Experimentation (CMRE)                                                 *
//                                                                        *
// This is free software: you can redistribute it and/or modify it        *
// under the terms of the GNU General Public License version 3 as         *
// published by the Free Software Foundation.                             *
//                                                                        *
// This program is distributed in the hope that it will be useful, but    *
// WITHOUT ANY WARRANTY; without even the implied warranty of FITNESS     *
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for       *
// more details.                                                          *
//                                                                        *
// You should have received a copy of the GNU General Public License      *
// along with this program. If not, see <http://www.gnu.org/licenses/>.   *
//*************************************************************************
// Author: Ricardo Martins                                                *
//*************************************************************************

// ISO C headers.
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

// JANUS headers.
#include <janus/constants.h>
#include <janus/defaults.h>
#include <janus/hop_index.h>
#include <janus/modulator.h>
#include <janus/utils/memory.h>
#include <janus/utils/hamming_window.h>

struct janus_modulator
{
  //! Primitive Alpha.
  unsigned prim_a;
  //! Primitive Q.
  unsigned prim_q;

  janus_real_t chip_len_factor;

  //! Hamming window.
  janus_real_t* ham_win;
  //! Current index.
  unsigned idx;
  //! Signal index.
  janus_complex_t* out;
  //! Output samples buffer size.
  unsigned out_size;

  //! Modulator last phasor.
  janus_complex_t phasor;
  //! Phasor increment.
  janus_complex_t* inc;
};

janus_modulator_t
janus_modulator_new(janus_pset_t pset, unsigned fs)
{
  janus_modulator_t mod = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_modulator, 1);
  
  janus_real_t fh = (JANUS_FLOOR(JANUS_CHIP_FRQ_COUNT / 2.0) * JANUS_ALPHABET_SIZE + 1.0) * pset->chip_frq;
  unsigned i;

  mod->idx = 0;
  mod->prim_a = pset->primitive.alpha;
  mod->prim_q = pset->primitive.q;
  mod->chip_len_factor = pset->chip_dur * fs;
  mod->out_size = (unsigned)JANUS_ROUND(mod->chip_len_factor) + 1;
  mod->out = JANUS_UTILS_MEMORY_NEW(janus_complex_t, mod->out_size);

  {
    // Compute FH-BFSK chip sequence parameters.
    unsigned lt = (unsigned)JANUS_ROUND(pset->chip_dur * fs);
    unsigned ham_len = lt / 8;
    unsigned part_len = lt / 16;
    unsigned ones_len = lt - 2 * part_len;
    unsigned win_len = (part_len * 2) + ones_len + 1;

    mod->ham_win = JANUS_UTILS_MEMORY_NEW_ZERO(janus_real_t, win_len);

    // Build the FH-BFSK chip sequence.
    janus_utils_hamming_window_part(ham_len, 0, part_len, mod->ham_win);
    janus_utils_hamming_window_part(ham_len, part_len, ham_len, mod->ham_win + part_len + ones_len);
    for (i = part_len; i < win_len - part_len; ++i)
      mod->ham_win[i] = 1.0;
  }

  mod->inc = JANUS_UTILS_MEMORY_NEW(janus_complex_t, mod->prim_q * JANUS_ALPHABET_SIZE);

  for (i = 0; i != mod->prim_q * JANUS_ALPHABET_SIZE; ++i)
  {
    // Frequency of the symbol.
    janus_hiprecision_t ftone = (int)(-fh + i * pset->chip_frq);

    janus_hiprecision_t phase_inc = JANUS_HP_2_PI * ftone / fs;

    // Computing phase increment for each tone frequency.
    janus_complex_new(JANUS_COS(phase_inc), JANUS_SIN(phase_inc), mod->inc[i]);
  }

  janus_modulator_reset(mod);

  return mod;
}

void
janus_modulator_free(janus_modulator_t mod)
{
  JANUS_UTILS_MEMORY_FREE(mod->inc);
  JANUS_UTILS_MEMORY_FREE(mod->out);
  JANUS_UTILS_MEMORY_FREE(mod->ham_win);
  JANUS_UTILS_MEMORY_FREE(mod);
}

unsigned
janus_modulator_execute(janus_modulator_t mod, janus_uint8_t chip, janus_complex_t** out)
{
  // Adapt chip length.
  unsigned chip_len = (unsigned)(JANUS_ROUND((mod->idx + 1) * mod->chip_len_factor) - JANUS_ROUND(mod->idx * mod->chip_len_factor));

  // Get hop index.
  unsigned hop_idx = janus_hop_index(mod->idx, mod->prim_a, mod->prim_q);

  unsigned i, k;

  janus_complex_t p;

  janus_complex_assign(mod->phasor, p);

  if (*out == NULL)
    *out = mod->out;

  k = hop_idx * JANUS_ALPHABET_SIZE + chip;

  // Waveform of the single chip.
  for (i = 0; i < chip_len; ++i)
  {
    janus_real_t amc, bmd, apbmcpd;

    amc = (p[0] * (mod->inc[k])[0]);
    bmd = (p[1] * (mod->inc[k])[1]);
    apbmcpd = (p[0] + p[1]) * ((mod->inc[k])[0] + (mod->inc[k])[1]);
    p[0] = amc - bmd;
    p[1] = apbmcpd - amc - bmd;

    (*out)[i][0] = p[0] * mod->ham_win[i];
    (*out)[i][1] = p[1] * mod->ham_win[i];
  }

  ++(mod->idx);

  janus_complex_mul_real(p, JANUS_REAL_CONST(1.0) / janus_complex_abs(p), p); 
  janus_complex_assign(p, mod->phasor);

  return chip_len;
}

void
janus_modulator_reset(janus_modulator_t mod)
{
  mod->idx = 0;
  janus_complex_new(JANUS_REAL_CONST(1.0), JANUS_REAL_CONST(0.0), mod->phasor);
}

unsigned
janus_modulator_get_max_samples(janus_modulator_t mod)
{
  return mod->out_size;
}
