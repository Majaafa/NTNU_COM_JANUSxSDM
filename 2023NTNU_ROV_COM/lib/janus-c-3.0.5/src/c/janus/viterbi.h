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

#ifndef JANUS_VITERBI_H_INCLUDED_
#define JANUS_VITERBI_H_INCLUDED_

// JANUS headers.
#include <janus/export.h>
#include <janus/trellis.h>

//! @ingroup CONV
//! @{

//! Viterbi decoder object.
typedef struct janus_viterbi* janus_viterbi_t;

//! Create a Viterbi decoder object.
//! @param trellis Trellis structure.
//! @param sdc_bits Number of soft decision bits.
//! @param tbk_depth traceback depth.
//! @return Viterbi decoder object.
JANUS_EXPORT janus_viterbi_t
janus_viterbi_new(janus_trellis_t trellis, unsigned sdc_bits, unsigned tbk_depth);

//! Free Viterbi decoder object.
//! @param vit Viterbi decoder object.
JANUS_EXPORT void
janus_viterbi_free(janus_viterbi_t vit);

//! Decode sequence.
//! @param vit Viterbi decoder object.
//! @param inp input sequence.
//! @param inp_len size of input sequence.
//! @param out decoded sequence.
//! @param tbk_depth traceback depth.
JANUS_EXPORT void
janus_viterbi_execute(janus_viterbi_t vit, const janus_uint8_t* inp, unsigned inp_len, janus_uint8_t* out, unsigned tbk_depth);

//! @}

#endif
