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

#ifndef JANUS_SIMPLE_RX_H_INCLUDED_
#define JANUS_SIMPLE_RX_H_INCLUDED_

// JANUS headers.
#include <janus/export.h>
#include <janus/types.h>
#include <janus/parameters.h>
#include <janus/packet.h>
#include <janus/rx_state.h>
#include <janus/pset.h>
#include <janus/stream/istream.h>
#include <janus/ddc.h>
#include <janus/rx.h>

//! @ingroup PACKET
//! @{

//! Simple receive object.
typedef struct janus_simple_rx* janus_simple_rx_t;

//! Create a simple receive object.
//! @param params parameters.
//! @return simple receive object.
JANUS_EXPORT janus_simple_rx_t
janus_simple_rx_new(janus_parameters_t params);

//! Free simple receive object.
//! @param simple_rx simple receive object.
JANUS_EXPORT void
janus_simple_rx_free(janus_simple_rx_t simple_rx);

//! Receive packet.
//! @param simple_rx simple receive object.
//! @param packet packet object.
//! @param state receive state.
JANUS_EXPORT int
janus_simple_rx_execute(janus_simple_rx_t simple_rx, janus_packet_t packet, janus_rx_state_t state);

//! Retrieve parameter set.
//! @param simple_rx simple receive object.
//! @return parameter set.
JANUS_EXPORT janus_pset_t
janus_simple_rx_get_pset(janus_simple_rx_t simple_rx);

//! Retrieve input stream.
//! @param simple_rx simple receive object.
//! @return input stream.
JANUS_EXPORT janus_istream_t
janus_simple_rx_get_istream(janus_simple_rx_t simple_rx);

//! Retrieve digital downconverter.
//! @param simple_rx simple receive object.
//! @return digital downconverter.
JANUS_EXPORT janus_ddc_t
janus_simple_rx_get_ddc(janus_simple_rx_t simple_rx);

//! Retrieve rx.
//! @param simple_rx simple receive object.
//! @return rx.
JANUS_EXPORT janus_rx_t
janus_simple_rx_get_rx(janus_simple_rx_t simple_rx);

//! Retrieve detection status.
//! @param simple_rx simple receive object.
//! @return 1 if simple_rx has decteted, 0 otherwise.
JANUS_EXPORT int
janus_simple_rx_has_detected(janus_simple_rx_t simple_rx);

//! Retrieve decoding status.
//! @param simple_rx simple receive object.
//! @return 1 if simple_rx is decoding, 0 otherwise.
JANUS_EXPORT int
janus_simple_rx_is_decoding(janus_simple_rx_t simple_rx);

//! Get the triggering detection time.
//! @param simple_rx simple receive object.
//! @return the triggering detection time.
JANUS_EXPORT janus_real_t
janus_simple_rx_get_first_detection_time(janus_simple_rx_t simple_rx);

//! Get the detection time.
//! @param simple_rx simple receive object.
//! @return the detection time.
JANUS_EXPORT janus_real_t
janus_simple_rx_get_detection_time(janus_simple_rx_t simple_rx);

//! Get the goertzel frequencies.
//! @param simple_rx simple receive object.
//! @param goertzel frequencies vector.
//! @param goertzel frequencies vector size.
JANUS_EXPORT void
janus_simple_rx_get_frequencies(janus_simple_rx_t simple_rx, unsigned* freq_vector_size, janus_real_t** freq_vector);

//! Get the in band spectrogram.
//! @param simple_rx simple receive object.
//! @param spectogram matrix of (26 lines) which can be reallocated.
//! @param valid how many columns are valid
//! @param last_time of most recent value
//! @param step_time time interval betwen time samples
//! @return 1 if ok otherwise 0 if busy decoding.
JANUS_EXPORT int
janus_simple_rx_get_channel_spectrogram(janus_simple_rx_t simple_rx, janus_real_t** spectrogram, unsigned* valid, janus_real_t* last_time, janus_real_t* step_time);

//! @}

#endif
