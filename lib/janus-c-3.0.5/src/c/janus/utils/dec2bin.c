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
#include <janus/janus.h>

void
janus_utils_dec2bin_byte(janus_uint8_t byte, janus_uint8_t* out)
{
  unsigned i = 0;

  for (i = 0; i < 8; ++i)
  {
    out[i] = ((byte & 0x80) ? 1 : 0);
    byte <<= 1;
  }
}

void
janus_utils_dec2bin(const janus_uint8_t* inp, unsigned inp_len, janus_uint8_t* out)
{
  unsigned i = 0;

  for (i = 0; i < inp_len; ++i)
  {
    janus_utils_dec2bin_byte(inp[i], out + (i * 8));
  }
}
