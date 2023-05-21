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

// JANUS headers.
#include <janus/janus.h>

#if defined(JANUS_WITH_PULSE)

#include <pulse/simple.h>
#include <pulse/error.h>

struct private_data
{
  // Code of last error.
  int error;

  pa_simple* fd;
};

static int
ostream_open(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  pa_sample_spec spec;
  memset(&spec, 0, sizeof(pa_sample_spec));

  switch (ostream->format)
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


  spec.channels = ostream->channel_count;
  spec.rate = ostream->fs;

  if (!(pdata->fd = pa_simple_new(NULL, "JANUS", PA_STREAM_PLAYBACK,
                                  NULL, "Signal", &spec, NULL,
                                  NULL, &pdata->error)))
    return JANUS_ERROR_STREAM;

  if (pa_simple_flush(pdata->fd, &pdata->error) < 0)
    return JANUS_ERROR_STREAM;

  return JANUS_ERROR_NONE;
}

static int
ostream_close(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  if (pdata->fd != NULL)
  {
    pa_simple_drain(pdata->fd, NULL);
    pa_simple_free(pdata->fd);
    pdata->fd = NULL;
  }

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
ostream_write(janus_ostream_t ostream, void* frame, unsigned frame_count)
{
  JANUS_OSTREAM_PDATA;

  int rv = pa_simple_write(pdata->fd, frame,
                           frame_count * ostream->frame_size,
                           &pdata->error);
  return rv;
}

static const char*
ostream_get_error(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;
  return pa_strerror(pdata->error);
}

static const char*
ostream_get_error_op(janus_ostream_t ostream)
{
  return NULL;
}

int
janus_ostream_pulse_new(janus_ostream_t ostream)
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
  strcpy(ostream->name, "PULSE");

  return JANUS_ERROR_NONE;
}

#endif
