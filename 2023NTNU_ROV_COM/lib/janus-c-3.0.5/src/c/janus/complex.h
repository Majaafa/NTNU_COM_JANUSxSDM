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

#ifndef JANUS_COMPLEX_H_INCLUDED_
#define JANUS_COMPLEX_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>

//! @ingroup TYPES
//! @{

//! Complex number.
typedef janus_real_t janus_complex_t[2];

//! New complex number.
//! @param a real part.
//! @param b imaginary part.
//! @return r result.
JANUS_EXPORT void
janus_complex_new(const janus_real_t a, const janus_real_t b, janus_complex_t c);

//! Assign complex number.
//! @param x x.
//! @return r result.
JANUS_EXPORT void
janus_complex_assign(const janus_complex_t x, janus_complex_t r);

//! Sum two complex numbers.
//! @param x x.
//! @param y y.
//! @param r result.
JANUS_EXPORT void
janus_complex_sum(const janus_complex_t x, const janus_complex_t y, janus_complex_t r);

//! Subtract two complex numbers.
//! @param x x.
//! @param y y.
//! @param r result.
JANUS_EXPORT void
janus_complex_sub(const janus_complex_t x, const janus_complex_t y, janus_complex_t r);

//! Multiply two complex numbers.
//! @param x x.
//! @param y y.
//! @param r result.
JANUS_EXPORT void
janus_complex_mul(const janus_complex_t x, const janus_complex_t y, janus_complex_t r);

//! Compute the absolute value of a complex number.
//! @param nr complex number.
//! @return absolute value.
JANUS_EXPORT janus_real_t
janus_complex_abs(const janus_complex_t nr);

//! Compute the absolute square value of a complex number.
//! @param nr complex number.
//! @return absolute square value.
JANUS_EXPORT janus_real_t
janus_complex_abs_sqr(const janus_complex_t nr);

//! Compute the complex conjugate of a complex number.
//! @param x x.
//! @param r result.
JANUS_EXPORT void
janus_complex_conj(const janus_complex_t x, janus_complex_t r);

//! Compute the imaginary exponential of a real number (exp(x * I)).
//! @param x x.
//! @param r result.
JANUS_EXPORT void
janus_complex_jexp(const janus_hiprecision_t x, janus_complex_t r);

//! Sum one complex number and one real number.
//! @param x x.
//! @param y y.
//! @param r result.
JANUS_EXPORT void
janus_complex_sum_real(const janus_complex_t x, const janus_real_t y, janus_complex_t r);

//! Subtract one complex number and one real number.
//! @param x x.
//! @param y y.
//! @param r result.
JANUS_EXPORT void
janus_complex_sub_real(const janus_complex_t x, const janus_real_t y, janus_complex_t r);

//! Multiply one complex number and one real number.
//! @param x x.
//! @param y y.
//! @param r result.
JANUS_EXPORT void
janus_complex_mul_real(const janus_complex_t x, const janus_real_t y, janus_complex_t r);

//! @}

#endif
