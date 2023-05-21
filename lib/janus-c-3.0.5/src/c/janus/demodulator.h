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

#ifndef JANUS_DEMODULATOR_H_INCLUDED_
#define JANUS_DEMODULATOR_H_INCLUDED_

// JANUS headers.
#include <janus/complex.h>
#include <janus/pset.h>
#include <janus/utils/fifo.h>
#include <janus/export.h>

//! @ingroup MODEM
//! @{

typedef struct janus_demodulator* janus_demodulator_t;

JANUS_EXPORT janus_demodulator_t
janus_demodulator_new(janus_pset_t pset, unsigned bband_fs, janus_real_t min_cfactor);

JANUS_EXPORT void
janus_demodulator_free(janus_demodulator_t dem);

JANUS_EXPORT void
janus_demodulator_reset(janus_demodulator_t dem);

JANUS_EXPORT unsigned
janus_demodulator_execute(janus_demodulator_t dem, janus_utils_fifo_t fifo, janus_uint8_t* rv, janus_real_t* bit_prob);

JANUS_EXPORT void
janus_demodulator_set_cfactor(janus_demodulator_t dem, janus_real_t cfactor);

JANUS_EXPORT void
janus_demodulator_set_index(janus_demodulator_t dem, unsigned index);

//! @}

#endif
