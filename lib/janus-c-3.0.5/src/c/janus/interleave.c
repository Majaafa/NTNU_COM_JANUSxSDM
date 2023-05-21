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

// ISO C headers.
#include <stdlib.h>

// JANUS headers.
#include <janus/interleave.h>
#include <janus/utils/primes.h>

static const unsigned c_prime_min = 5;

unsigned
janus_interleave_q(unsigned length)
{
  // Find the lowest prime number which is not a factor of length.
  // @fixme: there's something more to this.
  unsigned k = c_prime_min;
  unsigned prime = 0;

  while (1)
  {
    prime = janus_utils_primes_get_previous(k);
    if (((length / prime) < prime) && ((length % prime) != 0))
      break;
    ++k;
  }

  return prime;
}

void
janus_interleave(const janus_uint8_t* inp, unsigned inp_len, janus_uint8_t* out)
{
  unsigned prime = janus_interleave_q(inp_len);
  unsigned i = 0;
  unsigned n = 1;
  unsigned* perm = (unsigned*)malloc(sizeof(unsigned) * inp_len);

  // Build permutation table.
  perm[0] = 0;
  //the altered part of the code
  for (i = 1; i < inp_len; ++i)
  {
    if((perm[i-1]+prime)>=inp_len )
    {
      perm[i] = n;
      ++n;
    }
    else
    {
      perm[i] = (perm[i - 1] + prime) % inp_len;

    }
  //the code for the old sequence
  //if you want to use the original code can this be run instead of the if
  //perm[i] = (perm[i - 1] + prime) % inp_len;

  }

  // Perform permutation.
  for (i = 0; i < inp_len; ++i)
    out[i] = inp[perm[i]];
  
  free(perm);


}