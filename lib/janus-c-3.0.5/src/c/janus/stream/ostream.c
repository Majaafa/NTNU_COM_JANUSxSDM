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
#include <math.h>
#include <string.h>
#include <stdlib.h>

// JANUS headers.
#include <janus/types.h>
#include <janus/error.h>
#include <janus/dump.h>
#include <janus/constants.h>
#include <janus/stream/ostream.h>
#include <janus/utils/memory.h>

// Default buffer capacity.
#define DEFAULT_BUFFER_CAPACITY 1024

// Declare driver initialization functions.
#define OSTREAM(a)                                      \
  int janus_ostream_ ## a ## _new(janus_ostream_t);
#include "ostream.def"

static const char* s_drivers[] = {
#define OSTREAM(a) #a,
#include "ostream.def"
  NULL
};

const char**
janus_ostream_get_drivers(void)
{
  return s_drivers;
}

janus_ostream_t
janus_ostream_new(const char* driver, const char* args)
{
  janus_ostream_t ostream = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_ostream, 1);
  int rv = 0;

  strcpy(ostream->args, args);
  janus_ostream_set_channel_count(ostream, 1);
  janus_ostream_set_channel(ostream, 0);
  janus_ostream_set_fs(ostream, 44100);
  janus_ostream_set_format(ostream, JANUS_STREAM_FORMAT_UNKNOWN);
  janus_ostream_set_duc(ostream, NULL);
  janus_ostream_set_buffer_capacity(ostream, DEFAULT_BUFFER_CAPACITY);
  janus_ostream_set_amplitude_factor(ostream, JANUS_REAL_CONST(0.95));
  janus_ostream_set_multiple(ostream, 1);

  // Select appropriate driver.
#define OSTREAM(a)                                                      \
  if (strcmp(#a, driver) == 0) { rv = janus_ostream_ ## a ## _new(ostream); } else
#include "ostream.def"
  {
  rv = JANUS_ERROR_STREAM;
  }
  if (rv != JANUS_ERROR_NONE)
  {
    JANUS_UTILS_MEMORY_FREE(ostream);
    return NULL;
  }

  return ostream;
}

void
janus_ostream_free(janus_ostream_t ostream)
{
  if (ostream->free)
    ostream->free(ostream);
  JANUS_UTILS_MEMORY_FREE(ostream);
}

void
janus_ostream_set_channel_count(janus_ostream_t ostream, unsigned count)
{
  ostream->channel_count = count;
}

unsigned
janus_ostream_get_channel_count(janus_ostream_t ostream)
{
  return ostream->channel_count;
}

void
janus_ostream_set_channel(janus_ostream_t ostream, unsigned channel)
{
  ostream->channel = channel;
}

unsigned
janus_ostream_get_channel(janus_ostream_t ostream)
{
  return ostream->channel;
}

void
janus_ostream_set_fs(janus_ostream_t ostream, unsigned fs)
{
  ostream->fs = fs;
  ostream->ts = JANUS_REAL_CONST(1.0) / fs;
}

unsigned
janus_ostream_get_fs(janus_ostream_t ostream)
{
  return ostream->fs;
}

void
janus_ostream_set_format(janus_ostream_t ostream, janus_stream_format_t format)
{
  ostream->format = format;
  ostream->sample_size = janus_stream_format_get_sample_size(ostream->format);
  ostream->quantize = janus_stream_format_get_quantizer(format);
  ostream->quantize_complex = janus_stream_format_get_quantizer_complex(format);
}

janus_stream_format_t
janus_ostream_get_format(janus_ostream_t ostream)
{
  return ostream->format;
}

unsigned
janus_ostream_get_frame_size(janus_ostream_t ostream)
{
  return ostream->frame_size;
}

unsigned
janus_ostream_get_sample_size(janus_ostream_t ostream)
{
  return ostream->sample_size;
}

void
janus_ostream_set_duc(janus_ostream_t ostream, janus_duc_t duc)
{
  ostream->duc = duc;
}

void
janus_ostream_set_amplitude_factor(janus_ostream_t ostream, janus_real_t factor)
{
  ostream->amp_factor = factor;
}

void
janus_ostream_set_multiple(janus_ostream_t ostream, janus_uint8_t multiple)
{
  ostream->multiple = multiple;
  if (ostream->multiple < 1)
    ostream->multiple = 1;
}

void
janus_ostream_set_buffer_capacity(janus_ostream_t ostream, unsigned capacity)
{
  ostream->out_capacity = capacity;
  ostream->out_offset = 0;
  ostream->out_size = 0;
}

JANUS_EXPORT unsigned
janus_ostream_get_buffer_capacity(janus_ostream_t ostream)
{
  return ostream->out_capacity;
}

int
janus_ostream_open(janus_ostream_t ostream)
{
  int rv;

  if (!ostream->duc)
  {
    ostream->channel_count = 2;
  }

  ostream->frame_size = ostream->sample_size * ostream->channel_count;

  rv = ostream->open(ostream);
  if (rv != JANUS_ERROR_NONE)
    return JANUS_ERROR_STREAM;

  // Initialize output buffer.
  ostream->out = calloc(ostream->out_capacity, ostream->frame_size);
  ostream->out_size = 0;

  // Initialize DUC buffer.
  if (ostream->duc)
    ostream->duc_out = JANUS_UTILS_MEMORY_NEW(janus_real_t, ostream->out_capacity);

  return rv;
}

int
janus_ostream_close(janus_ostream_t ostream)
{
  janus_ostream_flush(ostream);
  free(ostream->out);
  JANUS_UTILS_MEMORY_FREE(ostream->duc_out);
  return ostream->close(ostream);
}

int
janus_ostream_reset(janus_ostream_t ostream)
{
  janus_ostream_flush(ostream);
  ostream->reset(ostream);
}

int
janus_ostream_status(janus_ostream_t ostream)
{
  return ostream->status(ostream);
}

int
janus_ostream_write(janus_ostream_t ostream, janus_complex_t* samples, unsigned sample_count)
{
  unsigned offset = 0;
  unsigned remaining = sample_count;

  while (remaining > 0)
  {
    unsigned available = ostream->out_capacity - ostream->out_size;
    unsigned size = JANUS_MIN(available, remaining);

    if (ostream->duc)
    {
      janus_duc_execute(ostream->duc, samples + offset, size, ostream->duc_out, ostream->amp_factor);
      ostream->quantize(ostream->duc_out, size,
                        (char*)ostream->out + ostream->out_offset,
                        ostream->channel_count, ostream->channel);
    }
    else
    {
      unsigned i;
      for (i = 0; i < sample_count; ++i)
      {
        janus_complex_mul_real(samples[i], ostream->amp_factor, samples[i]);
      }

      ostream->quantize_complex(samples + offset, size,
                                (char*)ostream->out + ostream->out_offset);
    }

    offset += size;
    remaining -= size;
    ostream->out_size += size;
    ostream->out_offset += size * ostream->frame_size;
    if (ostream->out_size == ostream->out_capacity)
    {
      int rv = rv = janus_ostream_flush(ostream);
      if (rv < 0)
      {
        ostream->out_size -= size;
        ostream->out_offset -= size * ostream->frame_size;
        return rv;
      }
    }
  }
  
  return 0;
}

int
janus_ostream_flush(janus_ostream_t ostream)
{
  if (ostream->out_size > 0)
  {
    int rv = 0;
    rv = ostream->write(ostream, ostream->out, ostream->out_size);
    if (rv < 0)
    {
      return rv;
    }
    else
    {
      ostream->out_size = 0;
      ostream->out_offset = 0;
    }
  }

  return 0;
}

const char*
janus_ostream_get_error(janus_ostream_t ostream)
{
  sprintf(ostream->bfr_error, "%s: %s", ostream->get_error_op(ostream),
          ostream->get_error(ostream));
  return ostream->bfr_error;
}

void
janus_ostream_dump(janus_ostream_t ostream)
{
  JANUS_DUMP("Ostream", "Driver", "%s", ostream->name);
  JANUS_DUMP("Ostream", "Driver Arguments", "%s", ostream->args);
  JANUS_DUMP("Ostream", "Upconversion", "%s", ostream->duc ? "enabled" : "disabled");
  JANUS_DUMP("Ostream", "Channel Count", "%d", ostream->channel_count);
  JANUS_DUMP("Ostream", "Format", "%s", janus_stream_format_get_name(ostream->format));
  JANUS_DUMP("Ostream", "Bits Per Sample", "%u", janus_stream_format_get_sample_size_bits(ostream->format));
  JANUS_DUMP("Ostream", "Bytes Per Sample", "%u", ostream->sample_size);
  JANUS_DUMP("Ostream", "Frame Size", "%u", ostream->frame_size);
  JANUS_DUMP("Ostream", "Sampling Frequency (Hz)", "%u", ostream->fs);
  JANUS_DUMP("Ostream", "Sampling Period (s)", "%0.6f", ostream->ts);
  JANUS_DUMP("Ostream", "Amplitude Factor", JANUS_REAL_FMT, ostream->amp_factor);
  JANUS_DUMP("Ostream", "Samples Forced Divisor", "%u", ostream->multiple);
}
