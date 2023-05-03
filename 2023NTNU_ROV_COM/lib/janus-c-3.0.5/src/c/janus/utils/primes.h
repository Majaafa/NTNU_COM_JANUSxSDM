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

#ifndef JANUS_UTILS_PRIMES_H_INCLUDED_
#define JANUS_UTILS_PRIMES_H_INCLUDED_

// JANUS headers.
#include <janus/export.h>

//! @defgroup UTILS Utilities

//! Test if a given number is prime.
//! @param nr number.
//! @return 1 if number is prime, 0 otherwise.
JANUS_EXPORT int
janus_utils_primes_is_prime(unsigned nr);

//! Compute the first prime number less or equal to a given number.
//! @param nr number.
//! @return prime number.
JANUS_EXPORT unsigned
janus_utils_primes_get_previous(unsigned nr);

//! @}

#endif
