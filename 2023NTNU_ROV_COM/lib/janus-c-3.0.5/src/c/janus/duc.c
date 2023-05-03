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
#include <janus/duc.h>
#include <janus/constants.h>
#include <janus/utils/memory.h>

struct janus_duc
{
  //! Modulator last phasor.
  janus_complex_t phasor;
  //! Phasor increment.
  janus_complex_t inc;
};

janus_duc_t
janus_duc_new(unsigned fs, unsigned cfreq)
{
  janus_duc_t duc = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_duc, 1);

  janus_hiprecision_t ts = JANUS_HIPRECISION_CONST(1.0) / fs;
  janus_hiprecision_t phase_inc = JANUS_HP_2_PI * cfreq * ts;
  janus_complex_new(JANUS_COS(phase_inc), JANUS_SIN(phase_inc), duc->inc);

  janus_duc_reset(duc);

  return duc;
}

void
janus_duc_free(janus_duc_t duc)
{
  JANUS_UTILS_MEMORY_FREE(duc);
}

void
janus_duc_execute(janus_duc_t duc, janus_complex_t* samples_in, unsigned samples_in_count, janus_real_t* samples_out, janus_real_t amp)
{
  unsigned i;
  janus_complex_t p;

  janus_complex_assign(duc->phasor, p);
  janus_complex_mul_real(p, amp, p);

  for (i = 0; i < samples_in_count; ++i)
  {
    janus_real_t amc, bmd, apbmcpd;

    samples_out[i] = (samples_in[i][0] * p[0]) - (samples_in[i][1] * p[1]);

    amc = (p[0] * duc->inc[0]);
    bmd = (p[1] * duc->inc[1]);
    apbmcpd = (p[0] + p[1]) * (duc->inc[0] + duc->inc[1]);

    p[0] = amc - bmd;
    p[1] = apbmcpd - amc - bmd;
  }

  janus_complex_mul_real(p, JANUS_REAL_CONST(1.0) / janus_complex_abs(p), p); 
  janus_complex_assign(p, duc->phasor);
}

void
janus_duc_reset(janus_duc_t duc)
{
  janus_complex_new(JANUS_REAL_CONST(1.0), JANUS_REAL_CONST(0.0), duc->phasor);
}
