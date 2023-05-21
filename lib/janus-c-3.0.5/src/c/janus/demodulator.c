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
#include <stdio.h>

// JANUS headers.
#include <janus/constants.h>
#include <janus/defaults.h>
#include <janus/complex.h>
#include <janus/pset.h>
#include <janus/hop_index.h>
#include <janus/demodulator.h>
#include <janus/stream/ostream.h>
#include <janus/utils/memory.h>
#include <janus/utils/tukey_window.h>

struct janus_demodulator
{
  // Chip index.
  unsigned chip_idx;
  // fixme: 
  janus_real_t factor;
  // fixme: 
  janus_real_t cfactor;
  // Tukey window.
  janus_real_t* tw;
  // Tukey window size.
  unsigned tw_len;
  //! Baseband sampling frequency (Hz).
  unsigned bband_fs;
  //! Baseband sampling period (s).
  janus_real_t bband_ts;
  // Parameter set.
  janus_pset_t pset;
  // Internal buffer.
  janus_complex_t* bfr;
};

janus_demodulator_t
janus_demodulator_new(janus_pset_t pset, unsigned bband_fs, janus_real_t min_cfactor)
{
  janus_demodulator_t dem = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_demodulator, 1);
  janus_real_t max_factor;

  dem->tw = NULL;
  dem->pset = pset;
  dem->bband_fs = bband_fs;
  dem->bband_ts = JANUS_REAL_CONST(1.0) / bband_fs;
  dem->bfr = NULL;

  max_factor = (2 - min_cfactor) * dem->pset->chip_dur * dem->bband_fs;

  dem->bfr = JANUS_UTILS_MEMORY_NEW(janus_complex_t, (unsigned)JANUS_CEIL(max_factor));
  dem->tw = JANUS_UTILS_MEMORY_NEW(janus_real_t, (unsigned)JANUS_CEIL(max_factor));

  janus_demodulator_set_cfactor(dem, JANUS_REAL_CONST(1.0));
  janus_demodulator_reset(dem);

  return dem;
}

void
janus_demodulator_free(janus_demodulator_t dem)
{
  if (dem->tw)
  {
    JANUS_UTILS_MEMORY_FREE(dem->tw);
    dem->tw = NULL;
  }

  JANUS_UTILS_MEMORY_FREE(dem->bfr);
  JANUS_UTILS_MEMORY_FREE(dem);
}

void
janus_demodulator_reset(janus_demodulator_t dem)
{
  dem->chip_idx = 0;
}

unsigned
janus_demodulator_execute(janus_demodulator_t dem, janus_utils_fifo_t fifo, janus_uint8_t* rv, janus_real_t* bit_prob)
{
  unsigned hop_idx = janus_hop_index(dem->chip_idx, dem->pset->primitive.alpha, dem->pset->primitive.q) * 2;
  janus_complex_t a = {0};
  janus_complex_t b = {0};
  janus_complex_t tmp = {0};
  janus_complex_t win = {0};
  janus_hiprecision_t frq_v1 = 0;
  janus_hiprecision_t frq_v2 = 0;
  janus_hiprecision_t tw_v = 0;
  janus_real_t fh = 0;
  unsigned size = (unsigned)(JANUS_ROUND((dem->chip_idx + 1) * dem->factor) - JANUS_ROUND(dem->chip_idx * dem->factor));
  unsigned i;

  if (janus_utils_fifo_get_size(fifo) < (size * sizeof(janus_complex_t)))
    return 0;

  janus_utils_fifo_get(fifo, dem->bfr, size * sizeof(janus_complex_t));

#if 0
  {
    int j = 0;
    for (j = 0; j < size; ++j)
    {
      //printf("%0.8f, %0.8f\n", dem->bfr[j][0], dem->bfr[j][1]);
      //printf(" % .15f %+.15fi\n", dem->bfr[j][0], dem->bfr[j][1]);
      printf("abs: %f%s\n", janus_complex_abs(dem->bfr[j]), (janus_complex_abs(dem->bfr[j]) < 0.2 ? " *" : ""));
    }
  }
#endif

  fh = (JANUS_FLOOR(JANUS_CHIP_FRQ_COUNT / 2.0) * JANUS_ALPHABET_SIZE + 1.0) * dem->pset->chip_frq;

  frq_v1 = - fh + hop_idx * dem->pset->chip_frq;
  frq_v1 = -JANUS_HP_2_PI * ((dem->pset->cfreq + frq_v1) * dem->cfactor - dem->pset->cfreq) * dem->bband_ts;

  frq_v2 = - fh + (hop_idx + 1)* dem->pset->chip_frq;
  frq_v2 =  -JANUS_HP_2_PI *((dem->pset->cfreq + frq_v2) * dem->cfactor - dem->pset->cfreq) * dem->bband_ts;
  
  for (i = 0; i < size; ++i)
  {
    tw_v = frq_v1 * i;
    janus_complex_jexp(tw_v, tmp);
    janus_complex_mul_real(tmp, dem->tw[i], win);

    janus_complex_mul(dem->bfr[i], win, tmp);
    janus_complex_sum(a, tmp, a);

    tw_v = frq_v2 * i;
    janus_complex_jexp(tw_v, tmp);
    janus_complex_mul_real(tmp, dem->tw[i], win);
    
    janus_complex_mul(dem->bfr[i], win, tmp);
    janus_complex_sum(b, tmp, b);
  }
  
  ++dem->chip_idx;

  {
    // Compute absolute value and normalize.
    janus_real_t sum, bp;
    janus_real_t am = janus_complex_abs(a) / size;
    janus_real_t bm = janus_complex_abs(b) / size;

    am *= am;
    bm *= bm;

    // Convert to probability metric.
    sum = am + bm;
    bp = (((bm / sum) - (am / sum) + JANUS_REAL_CONST(1.0)) / JANUS_REAL_CONST(2.0));

    *rv = (janus_uint8_t)JANUS_FLOOR(bp * 256.0);

    if (bit_prob != 0)
    {
      *bit_prob = bp;
    }

#if 0
    printf("  % .15f\n", bp);
#endif
  }

  return size;
}

void
janus_demodulator_set_cfactor(janus_demodulator_t dem, janus_real_t cfactor)
{
  dem->cfactor = cfactor;
  dem->factor = (2 - cfactor) * dem->pset->chip_dur * dem->bband_fs;

  dem->tw_len = (unsigned)JANUS_CEIL(dem->factor);
  janus_utils_tukey_window(dem->tw_len, JANUS_TUKEY_RATIO, dem->tw);
}

void
janus_demodulator_set_index(janus_demodulator_t dem, unsigned index)
{
  dem->chip_idx = index;
}
