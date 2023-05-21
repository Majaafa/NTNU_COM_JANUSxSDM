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

#ifndef JANUS_CARRIER_SENSING_H_INCLUDED_
#define JANUS_CARRIER_SENSING_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>
#include <janus/rx.h>
#include <janus/utils/fifo.h>

//! Carrier sensing opaque object.
typedef struct janus_carrier_sensing* janus_carrier_sensing_t;

JANUS_EXPORT janus_carrier_sensing_t
janus_carrier_sensing_new(janus_rx_t rx);

JANUS_EXPORT void
janus_carrier_sensing_free(janus_carrier_sensing_t carrier_sensing);

JANUS_EXPORT int
janus_carrier_sensing_execute(janus_carrier_sensing_t carrier_sensing, janus_real_t* time);

JANUS_EXPORT void
janus_carrier_sensing_reset(janus_carrier_sensing_t carrier_sensing);

JANUS_EXPORT janus_real_t
janus_carrier_sensing_background_power(janus_carrier_sensing_t carrier_sensing);

JANUS_EXPORT janus_real_t
janus_carrier_sensing_window_power(janus_carrier_sensing_t carrier_sensing);

JANUS_EXPORT janus_real_t
janus_carrier_sensing_time(janus_carrier_sensing_t carrier_sensing);

#endif
