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

// Offset of the field Chunk Size.
#define CHUNK_SIZE_OFFS 4
// Offset of the field Subchunk2 Size.
#define DATA_SIZE_OFFS 40

struct private_data
{
  // Code of last error.
  int error;
  // Last error operation.
  const char* error_op;

  FILE* fd;
  unsigned size;
};

static int
ostream_open(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  // Scratch variables.
  janus_uint32_t null = 0;
  janus_uint32_t tmp4 = 0;
  janus_uint16_t tmp2 = 0;

  switch (ostream->format)
  {
    case JANUS_STREAM_FORMAT_S8:
    case JANUS_STREAM_FORMAT_S16:
    case JANUS_STREAM_FORMAT_S24:
    case JANUS_STREAM_FORMAT_S32:
      break;
    default:
      return JANUS_ERROR_STREAM;
  }

  pdata->fd = fopen(ostream->args, "wb");
  if (pdata->fd == NULL)
  {
    pdata->error_op = "opening file";
    pdata->error = errno;
    return JANUS_ERROR_STREAM;
  }

  pdata->size = 0;

  // Chunk Id.
  fwrite("RIFF", 4, 1, pdata->fd);
  // Chunk Size.
  fwrite(&null, 4, 1, pdata->fd);
  // Format.
  fwrite("WAVE", 4, 1, pdata->fd);
  // Subchunk1 Id.
  fwrite("fmt ", 4, 1, pdata->fd);
  // Subchunk1 Size.
  tmp4 = 16;
  fwrite(&tmp4, 4, 1, pdata->fd);
  // Subchunk1 Audio Format (PCM).
  tmp2 = 1;
  fwrite(&tmp2, 2, 1, pdata->fd);
  // Subchunk1 Number of Channels.
  tmp2 = ostream->channel_count;
  fwrite(&tmp2, 2, 1, pdata->fd);
  // Subchunk1 Sampling Frequency.
  tmp4 = ostream->fs;
  fwrite(&tmp4, 4, 1, pdata->fd);
  // Subchunk1 Byte Rate.
  tmp4 = (ostream->fs * ostream->frame_size);
  fwrite(&tmp4, 4, 1, pdata->fd);
  // Subchunk1 Block Align.
  tmp2 = ostream->frame_size;
  fwrite(&tmp2, 2, 1, pdata->fd);
  // Subchunk1 Bits Per Sample.
  tmp2 = janus_stream_format_get_sample_size_bits(ostream->format);
  fwrite(&tmp2, 2, 1, pdata->fd);
  // Subchunk2 Id.
  fwrite("data ", 4, 1, pdata->fd);
  // Subchunk2 Size.
  fwrite(&null, 4, 1, pdata->fd);

  return JANUS_ERROR_NONE;
}

static int
ostream_close(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  // Write remaining header info (data sizes).
  janus_uint32_t chunk_size = pdata->size + 36;
  fseek(pdata->fd, CHUNK_SIZE_OFFS, SEEK_SET);
  fwrite(&chunk_size, 4, 1, pdata->fd);
  fseek(pdata->fd, DATA_SIZE_OFFS, SEEK_SET);
  fwrite(&pdata->size, 4, 1, pdata->fd);

  // And we're done.
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

  pdata->size += ostream->frame_size * frame_count;

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
janus_ostream_wav_new(janus_ostream_t ostream)
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
  strcpy(ostream->name, "WAVE");

  return JANUS_ERROR_NONE;
}
