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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// JANUS headers.
#include <janus/utils/cfar.h>
#include <janus/utils/memory.h>

struct janus_cfar
{
  // Window size.
  unsigned n;
  // Half the window size.
  unsigned hn;
  // Guard size.
  unsigned g;
  // Half the guard size.
  unsigned hg;
  // Internal buffer.
  janus_real_t* bfr;
  // Internal buffer's size.
  unsigned bfr_size;
  // Internal buffer's size.
  unsigned bfr_size_bytes;
  // Threshold.
  janus_real_t t;
};

janus_cfar_t
janus_cfar_new(unsigned n, unsigned g, janus_real_t pfa)
{
  janus_cfar_t cfar = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_cfar, 1);

  if ((n % 2) != 0)
    ++n;

  cfar->n = n;
  cfar->hn = n / 2;

  if ((g % 2) != 0)
    ++g;

  cfar->g = g;
  cfar->hg = g / 2;
  cfar->t = JANUS_POW(pfa, (JANUS_REAL_CONST(-1.0) / n)) - 1;

  cfar->bfr_size = cfar->n;
  cfar->bfr_size_bytes = sizeof(janus_real_t) * cfar->bfr_size;
  cfar->bfr = (janus_real_t*)malloc(cfar->bfr_size_bytes);

  return cfar;
}

void
janus_cfar_free(janus_cfar_t cfar)
{
  free(cfar->bfr);
  JANUS_UTILS_MEMORY_FREE(cfar);
}

unsigned
janus_cfar_execute(janus_cfar_t cfar, janus_utils_fifo_t fifo)
{
  janus_real_t z, max = -1;
  unsigned i, idx = 0;

  while (janus_utils_fifo_get_size(fifo) >= cfar->bfr_size_bytes)
  {
    janus_utils_fifo_peek(fifo, cfar->bfr, cfar->bfr_size_bytes);
    janus_utils_fifo_skip(fifo, sizeof(janus_real_t));

    z = JANUS_REAL_CONST(0.0);
    
    for (i = 0; i < cfar->hn - cfar->hg; ++i)
      z += cfar->bfr[i];
    
    for (i = cfar->hn + cfar->hg; i < cfar->n; ++i)
      z += cfar->bfr[i];
    
    z *= cfar->t;

    for (i = cfar->hn - cfar->hg; i < cfar->hn + cfar->hg; ++i)
    {
      if (cfar->bfr[i] > z)
      {
        if (cfar->bfr[i] > max)
        {
          max = cfar->bfr[i];
          idx = i;
        }
      }
    }
    
    if (max > 0)
    {
      unsigned rv = (janus_utils_fifo_get_size(fifo) / sizeof(janus_real_t)) - idx;
      janus_utils_fifo_reset(fifo);
      return rv;
    }
  }

  return 0;
}
