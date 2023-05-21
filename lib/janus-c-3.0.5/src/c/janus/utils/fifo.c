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
#include <math.h>

// JANUS headers.
#include <janus/types.h>
#include <janus/utils/fifo.h>

struct janus_utils_fifo
{
  //! Internal buffer.
  char* data;
  //! Internal buffer's capacity (in bytes).
  unsigned capacity;
  //! Data is inserted at offset (in % capacity).
  unsigned in;
  //! Data is retrieved from offset (out % capacity).
  unsigned out;
};

static unsigned
fifo_min(unsigned a, unsigned b)
{
  if (a < b)
    return a;

  return b;
}

static unsigned
next_power_of_2(unsigned v)
{
  unsigned r = 1;
  
  while (r < v)
    r <<= 1;
  
  return r;
}

janus_utils_fifo_t
janus_utils_fifo_new(unsigned capacity)
{
  janus_utils_fifo_t fifo = (janus_utils_fifo_t)malloc(sizeof(struct janus_utils_fifo));

  if (capacity & (capacity - 1))
    fifo->capacity = next_power_of_2(capacity);
  else
    fifo->capacity = capacity;

  fifo->in = 0;
  fifo->out = 0;
  fifo->data = (char*)calloc(1, fifo->capacity);
  return fifo;
}

void
janus_utils_fifo_free(janus_utils_fifo_t fifo)
{
  free(fifo->data);
  free(fifo);
}

void
janus_utils_fifo_reset(janus_utils_fifo_t fifo)
{
  fifo->in = 0;
  fifo->out = 0;
}

unsigned
janus_utils_fifo_get_size(const janus_utils_fifo_t fifo)
{
  return fifo->in - fifo->out;
}

unsigned
janus_utils_fifo_get_capacity(const janus_utils_fifo_t fifo)
{
  return fifo->capacity;
}

unsigned
janus_utils_fifo_get_available(const janus_utils_fifo_t fifo)
{
  return janus_utils_fifo_get_capacity(fifo) - janus_utils_fifo_get_size(fifo);
}

unsigned
janus_utils_fifo_is_empty(const janus_utils_fifo_t fifo)
{
  return fifo->in == fifo->out;
}

unsigned
janus_utils_fifo_is_full(const janus_utils_fifo_t fifo)
{
  return janus_utils_fifo_get_capacity(fifo) == janus_utils_fifo_get_size(fifo);
}

unsigned
janus_utils_fifo_put(janus_utils_fifo_t fifo, const void* data, unsigned data_size)
{
  data_size = fifo_min(data_size, janus_utils_fifo_get_available(fifo));

  {
    // From fifo->in to the end.
    unsigned fsize = fifo_min(data_size, fifo->capacity - (fifo->in & (fifo->capacity - 1)));
    memcpy(fifo->data + (fifo->in & (fifo->capacity - 1)), data, fsize);
  
    // Starting from the beggining.
    memcpy(fifo->data, (char*)data + fsize, data_size - fsize);
  }

  fifo->in += data_size;
  
  return data_size;
}

unsigned
janus_utils_fifo_get(janus_utils_fifo_t fifo, void* data, unsigned data_size)
{
  data_size = janus_utils_fifo_peek(fifo, data, data_size);
  fifo->out += data_size;
  return data_size;
}

unsigned
janus_utils_fifo_peek(const janus_utils_fifo_t fifo, void* data, unsigned data_size)
{
  data_size = fifo_min(data_size, janus_utils_fifo_get_size(fifo));

  {
    // Get the data from "m_data", starting from "m_out" until the end.
    unsigned fsize = fifo_min(data_size, fifo->capacity - (fifo->out & (fifo->capacity - 1)));
    memcpy(data, fifo->data + (fifo->out & (fifo->capacity - 1)), fsize);

    // Get the rest (if any) from the beginning of "m_data"
    memcpy((char*)data + fsize, fifo->data, data_size - fsize);
  }
  
  return data_size;
}

unsigned
janus_utils_fifo_peek_offset(const janus_utils_fifo_t fifo, void* data, unsigned data_size, unsigned offset)
{
  data_size = fifo_min(data_size, janus_utils_fifo_get_size(fifo) - offset);

  {
    // Get the data from "m_data", starting from "m_out" until the end.
    unsigned fsize = fifo_min(data_size, fifo->capacity - ((fifo->out + offset) & (fifo->capacity - 1)));
    memcpy(data, fifo->data + ((fifo->out + offset) & (fifo->capacity - 1)), fsize);

    // Get the rest (if any) from the beginning of "m_data"
    memcpy((char*)data + fsize, fifo->data, data_size - fsize);
  }
  
  return data_size;
}

unsigned
janus_utils_fifo_skip(janus_utils_fifo_t fifo, unsigned size)
{
  size = fifo_min(size, janus_utils_fifo_get_size(fifo));
  fifo->out += size;
  return size;
}
