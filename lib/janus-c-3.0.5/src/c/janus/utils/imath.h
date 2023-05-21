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

#ifndef JANUS_UTILS_IMATH_H_INCLUDED_
#define JANUS_UTILS_IMATH_H_INCLUDED_

/* JANUS headers. */
#include <janus/export.h>

/** @ingroup IMATH */
/** @{ */

/**
 * Return the greater of two unsigned integers.
 * @param[in] x unsigned integer.
 * @param[in] y unsigned integer.
 * @return greater of the two arguments.
 */
static inline unsigned
janus_umax(unsigned x, unsigned y)
{
  return (x > y) ? x : y;
}

/**
 * Return the lesser of two unsigned integers.
 * @param[in] x unsigned integer.
 * @param[in] y unsigned integer.
 * @return lesser of the two arguments.
 */
static inline unsigned
janus_umin(unsigned x, unsigned y)
{
  return (x < y) ? x : y;
}

/**
 * Return the value of x raised to the power of y.
 * @param[in] x base.
 * @param[in] y exponent.
 * @return x raised to the power of y.
 */
static inline unsigned
janus_upow(unsigned x, unsigned y)
{
  unsigned tmp = x;

  if (y == 0)
    return 1;

  while (--y)
    x *= tmp;

  return x;
}

/**
 * Return the remainder after division of x by y.
 * @param[in] x dividend.
 * @param[in] y divisor.
 * @return remainder after division of x by y.
 */
static inline unsigned
janus_urem(unsigned x, unsigned y)
{
  return x - (x / y) * y;
}

/**
 * Divide x by y and return the smallest integral value that is not
 * less than (x/y).
 * @param[in] x dividend.
 * @param[in] y divisor.
 * @return x divided by y.
 */
static inline unsigned
janus_udiv_ceil(unsigned x, unsigned y)
{
  return 1 + (x - 1) / y;
}

/**
 * Compute exponentiation over a modulus:
 * (base ^ exponent) % modulus.
 *
 * @param[in] base base.
 * @param[in] exponent exponent.
 * @param[in] modulus modulus.
 * @return modular exponentiation.
 */
static inline unsigned
janus_mod_pow(unsigned base, unsigned exponent, unsigned modulus)
{
  unsigned result = 1;

  while (exponent > 0)
  {
    if (exponent & 1)
      result = (result * base) % modulus;
    base = (base * base) % modulus;
    exponent >>= 1;
  }

  return result;
}

/** @} */

#endif
