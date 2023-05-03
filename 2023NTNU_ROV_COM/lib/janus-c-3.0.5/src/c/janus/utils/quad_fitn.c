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
// Author: Luigi Elia D'Amaro                                             *
//*************************************************************************

#include <janus/utils/quad_fitn.h>

void
janus_utils_quad_fitn(const janus_real_t* x, const janus_real_t* y, const unsigned n, janus_real_t* a, janus_real_t* b)
{
  janus_real_t s1 = JANUS_REAL_CONST(0.0);
  janus_real_t s2 = JANUS_REAL_CONST(0.0);
  janus_real_t s3 = JANUS_REAL_CONST(0.0);
  janus_real_t s4 = JANUS_REAL_CONST(0.0);
  janus_real_t sy = JANUS_REAL_CONST(0.0);
  janus_real_t sxy = JANUS_REAL_CONST(0.0);
  janus_real_t sx2y = JANUS_REAL_CONST(0.0);
  janus_real_t d = JANUS_REAL_CONST(0.0);
  
  unsigned i;
  for (i = 0; i < n; ++i)
  {
    janus_real_t tmp;

    tmp = x[i];
    s1 += tmp; // sum of x(i)

    tmp *= x[i];
    s2 += tmp; // sum of x(i)^2

    sx2y += tmp * y[i]; // sum of x(i)^2 * y(i)

    tmp *= x[i];
    s3 += tmp; // sum of x(i)^3

    tmp *= x[i];
    s4 += tmp; // sum of x(i)^4
    
    sy += y[i]; // sum of y(i)
    
    sxy += x[i] * y[i]; // sum of x(i) * y(i)
  }

  d = (n * s2 * s4 - JANUS_POW(s1, 2) * s4 - n * JANUS_POW(s3, 2) + 2 * s1 * s2 * s3 - JANUS_POW(s2, 3));

  *a = (s1 * s3 * sy - JANUS_POW(s2, 2) * sy - n * s3 * sxy + s1 * s2 * sxy + n * s2 * sx2y - JANUS_POW(s1, 2) * sx2y) / d;

  *b = -(s1 * s4 * sy - s2 * s3 * sy - n * s4 * sxy + JANUS_POW(s2, 2) * sxy + n * s3 * sx2y - s1 * s2 * sx2y) / d;
}
