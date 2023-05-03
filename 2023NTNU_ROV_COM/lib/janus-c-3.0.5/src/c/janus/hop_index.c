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
#include <janus/types.h>
#include <janus/utils/imath.h>
#include <janus/hop_index.h>

unsigned
janus_hop_index(unsigned idx, unsigned alpha, unsigned block_size)
{
  unsigned u1 = janus_udiv_ceil(idx + 1, (block_size - 1) * block_size);
  unsigned u2 = (idx / (block_size - 1));
  unsigned gp = (idx % (block_size - 1)) + 1;
  unsigned b = janus_mod_pow(alpha, gp, block_size);
  return (b * (u1 + u2 * b)) % block_size;
}
