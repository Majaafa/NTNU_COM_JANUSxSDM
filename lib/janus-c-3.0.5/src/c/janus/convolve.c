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

// JANUS headers.
#include <janus/convolve.h>

void
janus_convolve(const janus_trellis_t trellis, const janus_uint8_t* inp, unsigned inp_len, janus_uint8_t* out)
{
  unsigned state = 0;
  unsigned idx = 0;
  unsigned n = 0;

  for (n = 0; n < inp_len; ++n)
  {
    idx = state + inp[n] * trellis->state_count;
    out[n << 1] = (trellis->outputs[idx] >> 1) & 1;
    out[(n << 1) + 1] = (trellis->outputs[idx] & 1);
    state = trellis->next_states[idx];
  }
}
