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

#include <janus/utils/bin2dec.h>

janus_uint8_t
janus_utils_bin2dec_byte(const janus_uint8_t* inp)
{
  janus_uint8_t byte = 0;

  byte |= inp[0] << 7;
  byte |= inp[1] << 6;
  byte |= inp[2] << 5;
  byte |= inp[3] << 4;
  byte |= inp[4] << 3;
  byte |= inp[5] << 2;
  byte |= inp[6] << 1;
  byte |= inp[7] << 0;

  return byte;
}

int
janus_utils_bin2dec(const janus_uint8_t* inp, unsigned inp_len, janus_uint8_t* out)
{
  unsigned i = 0;
  janus_uint8_t* out_ptr = out;

  for (; i < inp_len; i += 8)
  {
    *out_ptr = janus_utils_bin2dec_byte(inp + i);
    ++out_ptr;
  }

  return 0;
}
