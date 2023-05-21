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

#ifndef JANUS_SIMPLE_TX_H_INCLUDED_
#define JANUS_SIMPLE_TX_H_INCLUDED_

// JANUS headers.
#include <janus/export.h>
#include <janus/types.h>
#include <janus/parameters.h>
#include <janus/packet.h>
#include <janus/tx_state.h>
#include <janus/pset.h>
#include <janus/stream/ostream.h>
#include <janus/duc.h>
#include <janus/tx.h>

//! @ingroup PACKET
//! @{

//! Simple transmit object.
typedef struct janus_simple_tx* janus_simple_tx_t;

//! Create a simple transmit object.
//! @param params parameters.
//! @return simple transmit object.
JANUS_EXPORT janus_simple_tx_t
janus_simple_tx_new(janus_parameters_t params);

//! Free simple transmit object.
//! @param simple_tx simple transmit object.
JANUS_EXPORT void
janus_simple_tx_free(janus_simple_tx_t simple_tx);

//! Transmit packet.
//! @param simple_tx simple transmit object.
//! @param packet packet object.
//! @param state transmit state.
JANUS_EXPORT int
janus_simple_tx_execute(janus_simple_tx_t simple_tx, janus_packet_t packet, janus_tx_state_t state);

//! Retrieve parameter set.
//! @param simple_tx simple transmit object.
//! @return parameter set.
JANUS_EXPORT janus_pset_t
janus_simple_tx_get_pset(janus_simple_tx_t simple_tx);

//! Retrieve output stream.
//! @param simple_tx simple transmit object.
//! @return output stream.
JANUS_EXPORT janus_ostream_t
janus_simple_tx_get_ostream(janus_simple_tx_t simple_tx);

//! Retrieve output stream status.
//! @param simple_tx simple transmit object.
//! @return output stream status.
JANUS_EXPORT int
janus_simple_tx_get_ostream_status(janus_simple_tx_t simple_tx);

//! Retrieve digital upconverter.
//! @param simple_tx simple transmit object.
//! @return digital upconverter.
JANUS_EXPORT janus_duc_t
janus_simple_tx_get_duc(janus_simple_tx_t simple_tx);

//! Retrieve tx.
//! @param simple_tx simple transmit object.
//! @return tx.
JANUS_EXPORT janus_tx_t
janus_simple_tx_get_tx(janus_simple_tx_t simple_tx);

//! @}

#endif
