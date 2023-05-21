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
// Authors: Ricardo Martins, Luigi Elia D'Amaro                           *
//*************************************************************************

// JANUS headers.
#include <janus/complex.h>

void
janus_complex_new(const janus_real_t a, const janus_real_t b, janus_complex_t c)
{
  c[0] = a;
  c[1] = b;
}

void
janus_complex_assign(const janus_complex_t x, janus_complex_t r)
{
  r[0] = x[0];
  r[1] = x[1];
}

void
janus_complex_sum(const janus_complex_t x, const janus_complex_t y, janus_complex_t r)
{
  r[0] = x[0] + y[0];
  r[1] = x[1] + y[1];
}

void
janus_complex_sub(const janus_complex_t x, const janus_complex_t y, janus_complex_t r)
{
  r[0] = x[0] - y[0];
  r[1] = x[1] - y[1];
}

void
janus_complex_mul(const janus_complex_t x, const janus_complex_t y, janus_complex_t r)
{
  janus_real_t amc = (x[0] * y[0]);
  janus_real_t bmd = (x[1] * y[1]);
  janus_real_t apbmcpd = (x[0] + x[1]) * (y[0] + y[1]);

  r[0] = amc - bmd;
  r[1] = apbmcpd - amc - bmd;
}

janus_real_t
janus_complex_abs(const janus_complex_t nr)
{
  return JANUS_SQRT(nr[0] * nr[0] + nr[1] * nr[1]);
}

janus_real_t
janus_complex_abs_sqr(const janus_complex_t nr)
{
  return nr[0] * nr[0] + nr[1] * nr[1];
}

void
janus_complex_conj(const janus_complex_t x, janus_complex_t r)
{
  r[0] = x[0];
  r[1] = -x[1];
}

void
janus_complex_jexp(const janus_hiprecision_t x, janus_complex_t r)
{
  r[0] = JANUS_HP_COS(x);
  r[1] = JANUS_HP_SIN(x);
}

void
janus_complex_sum_real(const janus_complex_t x, const janus_real_t y, janus_complex_t r)
{
  r[0] = x[0] + y;
  r[1] = x[1];
}

void
janus_complex_sub_real(const janus_complex_t x, const janus_real_t y, janus_complex_t r)
{
  r[0] = x[0] - y;
  r[1] = x[1];
}

void
janus_complex_mul_real(const janus_complex_t x, const janus_real_t y, janus_complex_t r)
{
  r[0] = x[0] * y;
  r[1] = x[1] * y;
}
