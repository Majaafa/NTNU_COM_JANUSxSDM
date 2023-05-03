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

#ifndef JANUS_GOERTZEL_H_INCLUDED_
#define JANUS_GOERTZEL_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/complex.h>

//! Goertzel algorithm opaque object.
typedef struct janus_goertzel* janus_goertzel_t;

//! Create a Goertzel algorithm object.
//! @param fs sampling frequency.
//! @param freq_vector_size number of frequencies.
//! @param freq_vector frequencies.
//! @param n number of samples (window length).
//! @return Goertzel algorithm object.
JANUS_EXPORT janus_goertzel_t
janus_goertzel_new(unsigned fs, unsigned freq_vector_size, janus_real_t freq_vector[], unsigned n);

//! Free Goertzel algorithm object.
//! @param goertzel Goertzel algorithm object.
JANUS_EXPORT void
janus_goertzel_free(janus_goertzel_t goertzel);

//! Execute Goertzel algorithm.
//! @param goertzel Goertzel algorithm object.
//! @param x samples.
//! @param r return DTFT values.
JANUS_EXPORT void
janus_goertzel_execute(janus_goertzel_t goertzel, janus_complex_t x[], janus_complex_t r[]);

//! Execute Goertzel algorithm.
//! @param goertzel Goertzel algorithm object.
//! @param x samples.
//! @param r return modulus of DTFT values.
JANUS_EXPORT void
janus_goertzel_abs_execute(janus_goertzel_t goertzel, janus_complex_t x[], janus_real_t r[]);

#endif
