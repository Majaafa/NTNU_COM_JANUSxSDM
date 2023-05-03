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
#include <janus/deinterleave.h>
#include <janus/interleave.h>

void
janus_deinterleave(const janus_uint8_t* inp, unsigned inp_len, janus_uint8_t* out)
{
  unsigned q = janus_interleave_q(inp_len);
  unsigned i = 0;
  unsigned idx = 0;
  unsigned index = 1;

  out[0] = inp[0];

  for (i = 1; i < inp_len; ++i)
  {
    //the altered part of the code
    if((idx+q)>=inp_len)
    {
      idx = index;
      ++index;
      out[idx] = inp[i];
    }
    else
    {
      idx = ((idx + q) % inp_len);
      out[idx] = inp[i];
    }

/*  //the code for the old sequence
    //if you want to use the original code can this be run instead of the if
    idx = ((idx + q) % inp_len);
    out[idx] = inp[i];
*/
  }
}
