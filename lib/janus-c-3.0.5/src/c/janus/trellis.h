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

#ifndef JANUS_TRELLIS_H_INCLUDED_
#define JANUS_TRELLIS_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>

//! @ingroup CONV
//! @{

//! Trellis object.
typedef struct janus_trellis* janus_trellis_t;

struct janus_trellis
{
  //! Number of input symbols.
  unsigned inp_sym_count;
  //! Number of output symbols.
  unsigned out_sym_count;
  //! Number of states.
  unsigned state_count;
  //! Outputs for all combinations of current state and current input.
  const janus_uint8_t* outputs;
  //! Next states for all combinations of current state and current input.
  const janus_uint8_t* next_states;
};

//! Retrieve JANUS default trellis object.
//! @return JANUS default trellis object.
JANUS_EXPORT const janus_trellis_t
janus_trellis_default(void);

//! @}

#endif
