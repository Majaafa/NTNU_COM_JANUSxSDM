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

// ISO C headers.
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// JANUS headers.
#include <janus/defaults.h>
#include <janus/dump.h>
#include <janus/tx_state.h>
#include <janus/utils/memory.h>

janus_tx_state_t
janus_tx_state_new(int coded_symbols_enable)
{
  janus_tx_state_t tx_state = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_tx_state, 1);

  tx_state->coded_symbols_enabled = (coded_symbols_enable) ? 1 : 0;

  return tx_state;
}

void
janus_tx_state_free(janus_tx_state_t tx_state)
{
  JANUS_UTILS_MEMORY_FREE(tx_state->coded_symbols);
  JANUS_UTILS_MEMORY_FREE(tx_state);
}

void
janus_tx_state_dump(const janus_tx_state_t tx_state)
{
  JANUS_DUMP("State", "Parameter Set Id", "%u", tx_state->pset_id);
  JANUS_DUMP("State", "Parameter Set Name", "%s", tx_state->pset_name);
  JANUS_DUMP("State", "Center Frequency (Hz)", "%u", tx_state->cfreq);
  JANUS_DUMP("State", "Available Bandwidth (Hz)", "%u", tx_state->bwidth);
  JANUS_DUMP("State", "Chip Frequency (Hz)", "%u", tx_state->chip_frq);
  JANUS_DUMP("State", "Chip Duration (s)", "%0.6f", tx_state->chip_dur);

  if (tx_state->coded_symbols_size)
  {
    unsigned i;
    char* string_coded_symbols = JANUS_UTILS_MEMORY_ALLOCA(char, tx_state->coded_symbols_size * 2 + 1);
    string_coded_symbols[0] = '\0';
    for (i = 0; i < tx_state->coded_symbols_size; ++i)
      APPEND_FORMATTED(string_coded_symbols, "%u ", tx_state->coded_symbols[i]);

    JANUS_DUMP("State", "Coded Symbols (binary)", "%s", string_coded_symbols);

    JANUS_UTILS_MEMORY_ALLOCA_FREE(string_coded_symbols);
  }
}
