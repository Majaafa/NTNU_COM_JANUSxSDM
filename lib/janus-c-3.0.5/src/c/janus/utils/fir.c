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
#include <math.h>
#include <stdlib.h>

// JANUS headers.
#include <janus/constants.h>
#include <janus/utils/fir.h>
#include <janus/utils/memory.h>

janus_utils_fir_t
janus_utils_fir_new(unsigned n, janus_real_t wn)
{
  janus_real_t* b = NULL;
  janus_real_t gain = JANUS_REAL_CONST(0.0);
  unsigned i = 0;

  // Allocate object.
  janus_utils_fir_t fir = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_utils_fir, 1);
  fir->n = n;
  fir->wn = wn;
  fir->coefs_n = n + 1;
  fir->coefs = (janus_real_t*)malloc(sizeof(janus_real_t) * fir->coefs_n);

  // Design filter.
  {
    unsigned j = 0;
    unsigned odd = fir->coefs_n - (fir->coefs_n/2) * 2;
    unsigned nhlf = fir->coefs_n / 2;
    janus_real_t c0 = 0;
    janus_real_t c1 = 0;

    b = JANUS_UTILS_MEMORY_NEW(janus_real_t, fir->coefs_n / 2);

    for (i = 0; i < nhlf; ++i)
    {
      c0 = JANUS_PI * (i + 0.5 * (1 - odd));
      c1 = 2 * (wn / 2.0) * c0;
      b[i] = sin(c1) / c0;
    }

    for (i = 0, j = nhlf - 1; i < nhlf; ++i, --j)
    {
      fir->coefs[i] = b[j] * (0.54 - 0.46 * cos((JANUS_2_PI * i) / n));
      gain += fir->coefs[i];
    }

    for (i = nhlf, j = 0; i < fir->coefs_n; ++i, ++j)
    {
      fir->coefs[i] = b[j] * (0.54 - 0.46 * cos((JANUS_2_PI * i) / n));
      gain += fir->coefs[i];
    }
  }

  JANUS_UTILS_MEMORY_FREE(b);
  
  for (i = 0; i < fir->coefs_n; ++i)
    fir->coefs[i] /= gain;

  return fir;
}

void
janus_utils_fir_free(janus_utils_fir_t fir)
{
  free(fir->coefs);
  JANUS_UTILS_MEMORY_FREE(fir);
}
