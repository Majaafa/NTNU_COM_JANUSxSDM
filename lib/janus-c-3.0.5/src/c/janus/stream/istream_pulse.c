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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// JANUS headers.
#include <janus/janus.h>

#if defined(JANUS_WITH_PULSE)

// PULSE headers.
#include <pulse/simple.h>
#include <pulse/error.h>

struct private_data
{
  // Code of last error.
  int error;
  // Last error operation.
  const char* error_op;

  // PULSE handle.
  pa_simple* fd;
};

static int
istream_open(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;

  pa_sample_spec spec;
  memset(&spec, 0, sizeof(pa_sample_spec));

  switch (istream->format)
  {
    case JANUS_STREAM_FORMAT_FLOAT:
      spec.format = PA_SAMPLE_FLOAT32NE;
      break;

    case JANUS_STREAM_FORMAT_S32:
      spec.format = PA_SAMPLE_S32NE;
      break;

    case JANUS_STREAM_FORMAT_S24:
      spec.format = PA_SAMPLE_S24NE;
      break;

    case JANUS_STREAM_FORMAT_S24_32:
      spec.format = PA_SAMPLE_S24_32NE;
      break;

    case JANUS_STREAM_FORMAT_S16:
      spec.format = PA_SAMPLE_S16NE;
      break;

    default:
      break;
  }

  if (spec.format == 0)
    return JANUS_ERROR_STREAM;

  spec.channels = istream->channel_count;
  spec.rate = istream->fs;

  if (!(pdata->fd = pa_simple_new(NULL, "JANUS", PA_STREAM_RECORD,
                                  NULL, "Signal", &spec, NULL,
                                  NULL, &pdata->error)))
    return JANUS_ERROR_STREAM;

  return JANUS_ERROR_NONE;
}

static int
istream_close(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;

  if (pdata->fd != NULL)
  {
    pa_simple_free(pdata->fd);
    pdata->fd = NULL;
  }

  return JANUS_ERROR_NONE;
}

static void
istream_free(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;
  free(pdata);
}

static int
istream_read(const janus_istream_t istream, void* frames, unsigned frame_count)
{
  JANUS_ISTREAM_PDATA;

  unsigned frame_size = janus_istream_get_frame_size(istream);

  int error;
  if (pa_simple_read(pdata->fd, frames, frame_count * frame_size, &error) < 0)
  {
    pdata->error = error;
    return -1;
  }

  return frame_count;
}

static int
istream_write(janus_istream_t istream, void* frames, unsigned frame_count)
{
  return 0;
}

static const char*
istream_get_error(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;
  return pa_strerror(pdata->error);
}

static const char*
istream_get_error_op(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;
  return pdata->error_op;
}

int
janus_istream_pulse_new(janus_istream_t istream)
{
  istream->pdata = calloc(1, sizeof(struct private_data));
  istream->open = istream_open;
  istream->close = istream_close;
  istream->free = istream_free;
  istream->read = istream_read;
  istream->write = istream_write;
  istream->get_error = istream_get_error;
  istream->get_error_op = istream_get_error_op;
  strcpy(istream->name, "PULSE");

  return JANUS_ERROR_NONE;
}

#endif
