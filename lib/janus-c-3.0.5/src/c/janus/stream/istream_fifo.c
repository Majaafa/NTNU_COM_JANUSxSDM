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
// Author: Luigi Elia D'Amaro                                             *
//*************************************************************************

// ISO C headers.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

// JANUS headers.
#include <janus/janus.h>
#include <janus/utils/imath.h>
#include <janus/utils/memory.h>
#include <janus/utils/fifo.h>

struct private_data
{
  // Code of last error.
  int error;
  // Last error operation.
  const char* error_op;

  janus_utils_fifo_t fifo;
};

static int
istream_open(janus_istream_t istream)
{
  return JANUS_ERROR_NONE;
}

static int
istream_close(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;

  janus_utils_fifo_reset(pdata->fifo);

  return JANUS_ERROR_NONE;
}

static void
istream_free(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;
  janus_utils_fifo_free(pdata->fifo);
  free(pdata);
}

static int
istream_read(const janus_istream_t istream, void* frames, unsigned frame_count)
{
  JANUS_ISTREAM_PDATA;

  unsigned frame_size = janus_istream_get_frame_size(istream);

  unsigned rv = janus_utils_fifo_get(pdata->fifo, frames, frame_count * frame_size);

  frame_count = rv / frame_size;

  return frame_count;
}

static int
istream_write(janus_istream_t istream, void* frames, unsigned frame_count)
{
  JANUS_ISTREAM_PDATA;

  unsigned frame_size = janus_istream_get_frame_size(istream);

  unsigned rv = janus_utils_fifo_put(pdata->fifo, frames, frame_count * frame_size);

  frame_count = rv / frame_size;

  return frame_count;
}

static const char*
istream_get_error(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;
  return strerror(pdata->error);
}

static const char*
istream_get_error_op(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;
  return pdata->error_op;
}

int
janus_istream_fifo_new(janus_istream_t istream)
{
  istream->pdata = calloc(1, sizeof(struct private_data));
  istream->open = istream_open;
  istream->close = istream_close;
  istream->free = istream_free;
  istream->read = istream_read;
  istream->write = istream_write;
  istream->get_error = istream_get_error;
  istream->get_error_op = istream_get_error_op;
  strcpy(istream->name, "FIFO");

  {
    JANUS_ISTREAM_PDATA;
    unsigned fifo_size = atoi(istream->args);
    pdata->fifo = janus_utils_fifo_new(fifo_size);
  }

  return JANUS_ERROR_NONE;
}
