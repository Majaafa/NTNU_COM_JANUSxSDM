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

#ifndef JANUS_TX_STATE_H_INCLUDED_
#define JANUS_TX_STATE_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>

typedef struct janus_tx_state* janus_tx_state_t;

struct janus_tx_state
{
  janus_uint16_t pset_id;
  char* pset_name;
  unsigned cfreq;
  unsigned bwidth;
  unsigned chip_frq;
  janus_real_t chip_dur;

  int coded_symbols_enabled;
  janus_uint8_t* coded_symbols;
  unsigned coded_symbols_size;
};

JANUS_EXPORT janus_tx_state_t
janus_tx_state_new(int coded_symbols_enable);

JANUS_EXPORT void
janus_tx_state_free(janus_tx_state_t tx_state);

JANUS_EXPORT void
janus_tx_state_dump(const janus_tx_state_t tx_state);

#endif
