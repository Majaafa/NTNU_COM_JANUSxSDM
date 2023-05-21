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
// Authors: Ricardo Martins, Luigi Elia D'Amaro, Giovanni Zappa           *
//*************************************************************************

#ifndef JANUS_RX_H_INCLUDED_
#define JANUS_RX_H_INCLUDED_

// JANUS headers.
#include <janus/pset.h>
#include <janus/parameters.h>
#include <janus/packet.h>
#include <janus/rx_state.h>
#include <janus/stream/istream.h>
#include <janus/export.h>

//! @ingroup PACKET
//! @{

//! Receive object.
typedef struct janus_rx* janus_rx_t;

//! Create receive object.
//! @param pset parameter set object.
//! @param istream input stream object.
//! @return receive object.
JANUS_EXPORT janus_rx_t
janus_rx_new(janus_pset_t pset, janus_parameters_t params, janus_istream_t stream, janus_uint8_t verbose);

//! Free receive object.
//! @param rx receive object.
JANUS_EXPORT void
janus_rx_free(janus_rx_t rx);

//! Receive packet.
//! @param rx receive object.
//! @param packet packet object.
//! @param state receive state.
JANUS_EXPORT int
janus_rx_execute(janus_rx_t rx, janus_packet_t packet, janus_rx_state_t state);

JANUS_EXPORT void
janus_rx_reset(janus_rx_t rx);

//! Retrieve detection status.
//! @param rx receive object.
//! @return 1 if rx has decteted, 0 otherwise.
JANUS_EXPORT int
janus_rx_has_detected(janus_rx_t rx);

//! Retrieve decoding status.
//! @param rx receive object.
//! @return 1 if rx is decoding, 0 otherwise.
JANUS_EXPORT int
janus_rx_is_decoding(janus_rx_t rx);

//! Get the triggering detection time.
//! @param rx receive object.
//! @return the triggering detection time.
JANUS_EXPORT janus_real_t
janus_rx_get_first_detection_time(janus_rx_t rx);

//! Get the detection time.
//! @param rx receive object.
//! @return the detection time.
JANUS_EXPORT janus_real_t
janus_rx_get_detection_time(janus_rx_t rx);

JANUS_EXPORT janus_real_t
janus_rx_get_gamma(janus_rx_t rx);

JANUS_EXPORT janus_real_t
janus_rx_get_speed(janus_rx_t rx);

//! Get the goertzel frequencies.
//! @param rx receive object.
//! @param goertzel frequencies vector.
//! @param goertzel frequencies vector size.
JANUS_EXPORT void
janus_rx_get_frequencies(janus_rx_t rx, unsigned* freq_vector_size, janus_real_t** freq_vector);

//! Get the in band spectrogram.
//! @param rx receive object.
//! @param spectogram matrix of (26 lines) which can be reallocated.
//! @param valid how many columns are valid
//! @param last_time of most recent value
//! @param step_time time interval betwen time samples
//! @return 1 if ok otherwise 0 if busy decoding.
JANUS_EXPORT int
janus_rx_get_channel_spectrogram(janus_rx_t rx, janus_real_t** spectrogram, unsigned* valid, janus_real_t* last_time, janus_real_t* step_time);

JANUS_EXPORT janus_uint8_t
janus_rx_get_rssi(janus_rx_t rx);

JANUS_EXPORT janus_real_t
janus_rx_get_snr(janus_rx_t rx);

//! Set the threshold parameter.
//! @param rx receive object.
//! @param detection_threshold the detection threshold value.
JANUS_EXPORT void
janus_rx_set_threshold(janus_rx_t rx, janus_real_t detection_threshold);

//! @}

#endif
