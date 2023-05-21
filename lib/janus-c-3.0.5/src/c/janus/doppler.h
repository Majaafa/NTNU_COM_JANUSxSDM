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
// Authors: Giovanni Zappa, Luigi Elia D'Amaro                            *
//*************************************************************************

#ifndef JANUS_DOPPLER_H_INCLUDED_
#define JANUS_DOPPLER_H_INCLUDED_

// JANUS headers.
#include <janus/pset.h>
#include <janus/utils/fifo.h>
#include <janus/export.h>

//! Doppler object.
typedef struct janus_doppler* janus_doppler_t;

JANUS_EXPORT janus_doppler_t
janus_doppler_new(janus_uint8_t enable, janus_pset_t pset, janus_real_t freq_sec[], unsigned n_chips, unsigned bband_fs, janus_real_t max_speed_ms);

JANUS_EXPORT void
janus_doppler_free(janus_doppler_t doppler);

JANUS_EXPORT unsigned
janus_doppler_execute(janus_doppler_t doppler, janus_utils_fifo_t fifo, janus_real_t* gamma, janus_real_t* speed);

JANUS_EXPORT void
janus_doppler_reset(janus_doppler_t doppler);

JANUS_EXPORT janus_real_t
janus_doppler_get_min_gamma(janus_doppler_t doppler);

#endif
