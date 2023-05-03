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

// ISO C 99 headers.
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// JANUS headers.
#include <janus/error.h>
#include <janus/dump.h>
#include <janus/utils/memory.h>
#include <janus/stream/istream.h>

#define DEFAULT_DRV_IN_SIZE 1024

// Declare driver initialization functions.
#define ISTREAM(a)                                      \
  int janus_istream_ ## a ## _new(janus_istream_t);
#include "istream.def"

static const char* s_drivers[] = {
#define ISTREAM(a) #a,
#include "istream.def"
  NULL
};

const char**
janus_istream_get_drivers(void)
{
  return s_drivers;
}

janus_istream_t
janus_istream_new(const char* driver, const char* args)
{
  janus_istream_t istream = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_istream, 1);
  int rv = 0;

  strcpy(istream->args, args);
  janus_istream_set_fs(istream, 44100);
  janus_istream_set_channel_count(istream, 1);
  janus_istream_set_channel(istream, 0);
  janus_istream_set_format(istream, JANUS_STREAM_FORMAT_UNKNOWN);

  // Select appropriate driver.
#define ISTREAM(a)                                                      \
  if (strcmp(#a, driver) == 0) { rv = janus_istream_ ## a ## _new(istream); } else
#include "istream.def"
  {
  rv = JANUS_ERROR_STREAM;
  }

  if (rv != JANUS_ERROR_NONE)
  {
    JANUS_UTILS_MEMORY_FREE(istream);
    return NULL;
  }
  
  return istream;
}

void
janus_istream_free(janus_istream_t istream)
{
  istream->free(istream);
  JANUS_UTILS_MEMORY_FREE(istream);
}

void
janus_istream_set_channel_count(janus_istream_t istream, unsigned count)
{
  istream->channel_count = count;
}

unsigned
janus_istream_get_channel_count(janus_istream_t istream)
{
  return istream->channel_count;
}

void
janus_istream_set_channel(janus_istream_t istream, unsigned channel)
{
  istream->channel = channel;
}

unsigned
janus_istream_get_channel(janus_istream_t istream)
{
  return istream->channel;
}

void
janus_istream_set_fs(janus_istream_t istream, unsigned fs)
{
  istream->fs = fs;
  istream->ts = JANUS_REAL_CONST(1.0) / fs;
}

unsigned
janus_istream_get_fs(janus_istream_t istream)
{
  return istream->fs;
}

void
janus_istream_set_format(janus_istream_t istream, janus_stream_format_t format)
{
  istream->format = format;
  istream->sample_size = janus_stream_format_get_sample_size(format);
  istream->normalize = janus_stream_format_get_normalizer(format);
  istream->normalize_complex = janus_stream_format_get_normalizer_complex(format);
}

janus_stream_format_t
janus_istream_get_format(janus_istream_t istream)
{
  return istream->format;
}

unsigned
janus_istream_get_frame_size(janus_istream_t istream)
{
  return istream->frame_size;
}

unsigned
janus_istream_get_sample_size(janus_istream_t istream)
{
  return istream->sample_size;
}

void
janus_istream_set_ddc(janus_istream_t istream, janus_ddc_t ddc)
{
  istream->ddc = ddc;
}

unsigned
janus_istream_get_output_fs(janus_istream_t istream)
{
  if (istream->ddc)
    return janus_ddc_get_output_fs(istream->ddc);
  
  return istream->fs;
}

int
janus_istream_open(janus_istream_t istream)
{
  int rv;

  if (istream->format == JANUS_STREAM_FORMAT_UNKNOWN)
    return JANUS_ERROR_STREAM;
  
  if (!istream->ddc)
  {
    istream->channel_count = 2;
  }

  rv = istream->open(istream);
  if (rv != JANUS_ERROR_NONE)
    return JANUS_ERROR_STREAM;

  istream->frame_size = istream->sample_size * istream->channel_count;

  if (istream->ddc)
  {
    istream->ddc_in_size = janus_ddc_get_input_sample_count(istream->ddc);
    istream->ddc_in = JANUS_UTILS_MEMORY_NEW(janus_real_t, istream->ddc_in_size);
    istream->out_size = janus_ddc_get_output_sample_count(istream->ddc);
  }
  else
  {
    istream->ddc_in_size = DEFAULT_DRV_IN_SIZE;
    istream->out_size = DEFAULT_DRV_IN_SIZE;
  }

  istream->out = JANUS_UTILS_MEMORY_NEW(janus_complex_t, istream->out_size);
  istream->drv_in_size = istream->ddc_in_size;
  istream->drv_in = calloc(istream->drv_in_size * istream->channel_count, istream->sample_size);
  
  return rv;
}

int
janus_istream_close(janus_istream_t istream)
{
  if (istream->ddc)
  {
    JANUS_UTILS_MEMORY_FREE(istream->ddc_in);
  }
  JANUS_UTILS_MEMORY_FREE(istream->out);
  free(istream->drv_in);

  istream->close(istream);

  return JANUS_ERROR_NONE;
}

int
janus_istream_read(janus_istream_t istream, janus_complex_t** samples)
{
  int rv = istream->read(istream, istream->drv_in, istream->drv_in_size);
  
  if (rv <= 0)
  {
    if (rv == JANUS_ERROR_OVERRUN)
    {
      return JANUS_ERROR_OVERRUN;
    }
    return JANUS_ERROR_STREAM;
  }
  
  if (istream->ddc)
  {
    istream->normalize(istream->drv_in, rv, istream->ddc_in, istream->channel_count, istream->channel);

    // Pad with zeros if needed.
    if (rv < (int)istream->ddc_in_size)
      memset(istream->ddc_in + rv, 0, (istream->ddc_in_size - rv) * sizeof(janus_real_t));
    
    janus_ddc_execute(istream->ddc, istream->ddc_in, istream->out);

    rv = istream->out_size;
  }
  else
  {
    istream->normalize_complex(istream->drv_in, rv, istream->out);
  }

  *samples = istream->out;
  return rv;
}

const char*
janus_istream_get_error(janus_istream_t istream)
{
  sprintf(istream->bfr_error, "%s: %s", istream->get_error_op(istream),
          istream->get_error(istream));
  return istream->bfr_error;
}

void
janus_istream_dump(janus_istream_t istream)
{
  JANUS_DUMP("Istream", "Driver", "%s", istream->name);
  JANUS_DUMP("Istream", "Driver Arguments", "%s", istream->args);
  JANUS_DUMP("Istream", "Downconversion", "%s", istream->ddc ? "enabled" : "disabled");
  JANUS_DUMP("Istream", "Channel Count", "%d", istream->channel_count);
  JANUS_DUMP("Istream", "Format", "%s", janus_stream_format_get_name(istream->format));
  JANUS_DUMP("Istream", "Bits Per Sample", "%u", janus_stream_format_get_sample_size_bits(istream->format));
  JANUS_DUMP("Istream", "Bytes Per Sample", "%u", istream->sample_size);
  JANUS_DUMP("Istream", "Frame Size", "%u", istream->frame_size);
  JANUS_DUMP("Istream", "Sampling Frequency (Hz)", "%u", istream->fs);
  JANUS_DUMP("Istream", "Sampling Period (s)", "%e", istream->ts);
}
