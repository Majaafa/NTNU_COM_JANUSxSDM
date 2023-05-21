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
// Authors: Ricardo Martins, Luigi Elia D'Amaro                           *
//*************************************************************************

// ISO C headers.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

// JANUS headers.
#include <janus/janus.h>

struct private_data
{
  // Code of last error.
  int error;
  // Last error operation.
  const char* error_op;

  FILE* fd;
};

static int
ostream_open(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  pdata->fd = fopen(ostream->args, "wb");
  if (pdata->fd == NULL)
  {
    pdata->error_op = "opening file";
    pdata->error = errno;
    return JANUS_ERROR_STREAM;
  }

  return JANUS_ERROR_NONE;
}

static int
ostream_close(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  fflush(pdata->fd);
  fclose(pdata->fd);

  return 0;
}

static int
ostream_reset(janus_ostream_t ostream)
{
  //JANUS_OSTREAM_PDATA;
  
  // Dummy function.
  
  return JANUS_ERROR_NONE;
}

static int
ostream_status(janus_ostream_t ostream)
{
  //JANUS_OSTREAM_PDATA;
  
  // Dummy function.
  
  return JANUS_ERROR_NONE;
}

static void
ostream_free(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;
  free(pdata);
}

static int
ostream_read(const janus_ostream_t ostream, void* frames, unsigned frame_count)
{
  return 0;
}

static int
ostream_write(janus_ostream_t ostream, void* frames, unsigned frame_count)
{
  JANUS_OSTREAM_PDATA;

  int rv = (int)fwrite(frames, ostream->frame_size, frame_count, pdata->fd);
  if (rv <= 0)
    return JANUS_ERROR_STREAM;

  if ((unsigned)rv != frame_count)
    return JANUS_ERROR_STREAM;

  return rv;
}

static const char*
ostream_get_error(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;
  return strerror(pdata->error);
}

static const char*
ostream_get_error_op(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;
  return pdata->error_op;
}

int
janus_ostream_raw_new(janus_ostream_t ostream)
{
  ostream->pdata = calloc(1, sizeof(struct private_data));
  ostream->open = ostream_open;
  ostream->close = ostream_close;
  ostream->reset = ostream_reset;
  ostream->status = ostream_status;
  ostream->free = ostream_free;
  ostream->read = ostream_read;
  ostream->write = ostream_write;
  ostream->get_error = ostream_get_error;
  ostream->get_error_op = ostream_get_error_op;
  strcpy(ostream->name, "RAW");

  return JANUS_ERROR_NONE;
}
