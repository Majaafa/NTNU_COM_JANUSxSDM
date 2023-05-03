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
  janus_uint32_t data_size;
};

static int
istream_open(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;

  // Scratch variables.
  char str[5];
  janus_uint32_t null = 0;
  janus_uint32_t tmp4 = 0;
  janus_uint16_t tmp2 = 0;

  switch (istream->format)
  {
    case JANUS_STREAM_FORMAT_S8:
    case JANUS_STREAM_FORMAT_S16:
    case JANUS_STREAM_FORMAT_S24:
    case JANUS_STREAM_FORMAT_S32:
      break;
    default:
      return JANUS_ERROR_STREAM;
  }

  pdata->fd = fopen(istream->args, "rb");
  if (pdata->fd == NULL)
  {
    pdata->error_op = "opening file";
    pdata->error = errno;
    return JANUS_ERROR_STREAM;
  }
  
  pdata->data_size = 0;
  
  // Chunk Id.
  if (fread(str, 1, 4, pdata->fd) != 4)
    return JANUS_ERROR_STREAM;
  
  if (memcmp("RIFF", str, 4) != 0)
    return JANUS_ERROR_STREAM;

  // Chunk Size.
  if (fread(&null, 4, 1, pdata->fd) != 1)
    return JANUS_ERROR_STREAM;
  
  // Format.
  if (fread(str, 1, 4, pdata->fd) != 4)
    return JANUS_ERROR_STREAM;

  if (memcmp("WAVE", str, 4) != 0)
    return JANUS_ERROR_STREAM;

  // Format Chunk: Chunk Id.
  if (fread(str, 1, 4, pdata->fd) != 4)
    return JANUS_ERROR_STREAM;

  if (memcmp("fmt ", str, 4) != 0)
    return JANUS_ERROR_STREAM;

  // Format Chunk: Chunk Size.
  if (fread(&tmp4, 4, 1, pdata->fd) != 1)
    return JANUS_ERROR_STREAM;

  if (tmp4 != 16)
    return JANUS_ERROR_STREAM;

  // Format Chunk: Compression Code.
  if (fread(&tmp2, 2, 1, pdata->fd) != 1)
    return JANUS_ERROR_STREAM;

  if (tmp2 != 1)
    return JANUS_ERROR_STREAM;

  // Format Chunk: Number of Channels.
  if (fread(&tmp2, 2, 1, pdata->fd) != 1)
    return JANUS_ERROR_STREAM;

  if (tmp2 != istream->channel_count)
    return JANUS_ERROR_STREAM;

  // Format Chunk: Sampling Frequency.
  if (fread(&tmp4, 4, 1, pdata->fd) != 1)
    return JANUS_ERROR_STREAM;

  if (tmp4 != istream->fs)
    return JANUS_ERROR_STREAM;

  // Format Chunk: Average Bytes Per Second.
  if (fread(&null, 4, 1, pdata->fd) != 1)
    return JANUS_ERROR_STREAM;

  // Format Chunk: Block Align.
  if (fread(&tmp2, 2, 1, pdata->fd) != 1)
    return JANUS_ERROR_STREAM;

  // Format Chunk: Bits per Sample.
  if (fread(&tmp2, 2, 1, pdata->fd) != 1)
    return JANUS_ERROR_STREAM;

  if (janus_stream_format_get_sample_size_bits(istream->format) != tmp2)
  {
    janus_stream_format_t format;
    sprintf(str, "S%u", tmp2);
    format = janus_stream_format_parse(str);
    if (format == JANUS_STREAM_FORMAT_UNKNOWN)
    {
      return JANUS_ERROR_STREAM;
    }
    janus_istream_set_format(istream, format);
    fprintf(stderr, "WARNING: forcing bit per sample to %u (read from WAV header).\n", tmp2);
  }

  // Data Chunk: Chunk Id.
  if (fread(str, 1, 4, pdata->fd) != 4)
    return JANUS_ERROR_STREAM;

  if (memcmp("data", str, 4) != 0)
    return JANUS_ERROR_STREAM;

  // Data Chunk: Chunk Size.
  if (fread(&pdata->data_size, 4, 1, pdata->fd) != 1)
    return JANUS_ERROR_STREAM;

  return JANUS_ERROR_NONE;
}

static int
istream_close(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;

  fclose(pdata->fd);

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
  
  int rv = (int)fread(frames, janus_istream_get_frame_size(istream), frame_count, pdata->fd); 
  if (rv <= 0)
    return JANUS_ERROR_STREAM;
  
  return rv;
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
  return strerror(pdata->error);
}

static const char*
istream_get_error_op(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;
  return pdata->error_op;
}

int
janus_istream_wav_new(janus_istream_t istream)
{
  istream->pdata = calloc(1, sizeof(struct private_data));
  istream->open = istream_open;
  istream->close = istream_close;
  istream->free = istream_free;
  istream->read = istream_read;
  istream->write = istream_write;
  istream->get_error = istream_get_error;
  istream->get_error_op = istream_get_error_op;
  strcpy(istream->name, "WAVE");

  return JANUS_ERROR_NONE;
}
