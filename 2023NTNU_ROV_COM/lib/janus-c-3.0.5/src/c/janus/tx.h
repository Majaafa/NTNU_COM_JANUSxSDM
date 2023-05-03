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

#ifndef JANUS_TX_H_INCLUDED_
#define JANUS_TX_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/pset.h>
#include <janus/packet.h>
#include <janus/tx_state.h>
#include <janus/export.h>
#include <janus/stream/ostream.h>

//! @ingroup PACKET
//! @{

//! Transmit object.
typedef struct janus_tx* janus_tx_t;

//! Create a transmit object.
//! @param pset parameter set object.
//! @param ostream output stream object.
//! @return transmit object.
JANUS_EXPORT janus_tx_t
janus_tx_new(janus_pset_t pset, janus_ostream_t ostream, janus_uint8_t verbose);

//! Free transmit object.
//! @param tx transmit object.
JANUS_EXPORT void
janus_tx_free(janus_tx_t tx);

//! Transmit packet.
//! @param tx transmit object.
//! @param packet packet object.
//! @param state transimt state.
JANUS_EXPORT int
janus_tx_execute(janus_tx_t tx, janus_packet_t packet, janus_tx_state_t state);

//! Enable/disable padding of transmitted signal.
//! @param tx transmit object.
//! @param enabled true to enable, false to disable.
JANUS_EXPORT void
janus_tx_set_padding(janus_tx_t tx, janus_uint8_t enabled);

//! Enable/disable generation of wake-up tones.
//! @param tx transmit object.
//! @param enabled 1 to enable, 0 to disable.
JANUS_EXPORT void
janus_tx_set_wake_up_tones(janus_tx_t tx, janus_uint8_t enabled);

//! Set the stream amplification level.
//! @param tx transmit object.
//! @param stream_amp stream amplification level.
JANUS_EXPORT void
janus_tx_set_stream_amp(janus_tx_t tx, janus_real_t stream_amp);

//! @}

#endif
