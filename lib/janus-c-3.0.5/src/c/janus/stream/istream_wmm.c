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

#if defined(JANUS_WITH_WMM)

// Microsoft Windows headers.
#include <windows.h>
#include <mmreg.h>
#include <mmsystem.h>
#include <ks.h>
#include <ksmedia.h>

#define BUFFER_COUNT        128
#define ERR_STR_BUFFER_SIZE 128

struct private_data
{
  // Code of last error.
  int error;
  // Last error operation.
  const char* error_op;

  // Wave out handle.
  HWAVEIN fd;
  //! Error string buffer.
  char error_str[ERR_STR_BUFFER_SIZE];
  // Buffers.
  WAVEHDR** bfr;
  // Next buffer index.
  unsigned bfr_next_idx;
  // Event.
  HANDLE event;
};

static WAVEHDR*
istream_new_buffer(janus_istream_t istream, unsigned size)
{
  JANUS_ISTREAM_PDATA;

  MMRESULT rv;
  WAVEHDR* hdr = (WAVEHDR*)calloc(1, sizeof(WAVEHDR));

  hdr->dwLoops = 0;
  hdr->dwFlags = 0;
  hdr->lpData = (LPSTR)calloc(size, 1);
  hdr->dwBufferLength = size;
  hdr->dwBytesRecorded = 0;
  hdr->dwUser = 0;

  rv = waveInPrepareHeader(pdata->fd, hdr, sizeof(WAVEHDR));
  if (rv != MMSYSERR_NOERROR)
  {
    free(hdr->lpData);
    free(hdr);
    return 0;
  }

  waveInAddBuffer(pdata->fd, hdr, sizeof(WAVEHDR));

  return hdr;
}

static int
istream_open(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;

  MMRESULT rv;
  WAVEFORMATEX fmt;

  fmt.wFormatTag = WAVE_FORMAT_PCM;
  fmt.nChannels = istream->channel_count;
  fmt.nSamplesPerSec = istream->fs;
  fmt.wBitsPerSample = janus_stream_format_get_sample_size_bits(istream->format);
  fmt.nBlockAlign = istream->sample_size * istream->channel_count;
  fmt.nAvgBytesPerSec = fmt.nBlockAlign * fmt.nSamplesPerSec;
  fmt.cbSize = 0;

  pdata->event = CreateEvent(NULL,0,0,NULL);
  
  rv = waveInOpen(&pdata->fd,
                  atoi(istream->args),
                  &fmt,
                  (DWORD_PTR)pdata->event,
                  (DWORD_PTR)0,
                  CALLBACK_EVENT);
  
  if (rv != MMSYSERR_NOERROR)
  {
    pdata->error = rv;
    pdata->error_op = "opening input device";
    return JANUS_ERROR_STREAM;
  }

  // Allocate internal buffers.
  pdata->bfr = (WAVEHDR**)calloc(BUFFER_COUNT, sizeof(WAVEHDR*));

  {
    unsigned frame_count = 1024;
    int i;
    for (i = 0; i < BUFFER_COUNT; ++i)
    {
      pdata->bfr[i] = istream_new_buffer(istream, frame_count * fmt.nBlockAlign);
      if (pdata->bfr[i] == 0)
      {
        pdata->error = rv;
        pdata->error_op = "preparing waveform data block";
        return JANUS_ERROR_STREAM;
      }
    }
  }

  waveInStart(pdata->fd);
  ResetEvent(pdata->event);

  return JANUS_ERROR_NONE;
}

static int
istream_close(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;

  MMRESULT rv;
  int i;

  waveInStop(pdata->fd);
  rv = waveInClose(pdata->fd);

  for (i = 0; i < BUFFER_COUNT; ++i)
  {
    free(pdata->bfr[i]->lpData);
    free(pdata->bfr[i]);
  }
  free(pdata->bfr);

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

  unsigned rv = 0;
  unsigned idx = pdata->bfr_next_idx;

  while (1)
  {
    WaitForSingleObject(pdata->event, INFINITE);

    if (pdata->bfr[idx]->dwFlags & WHDR_DONE)
    {
      memcpy(frames, pdata->bfr[idx]->lpData, pdata->bfr[idx]->dwBufferLength);
      rv = waveInAddBuffer(pdata->fd, pdata->bfr[idx], sizeof(WAVEHDR));
      if (rv != MMSYSERR_NOERROR)
      {
        pdata->error = rv;
        pdata->error_op = "sending input buffer to input device";
        return JANUS_ERROR_STREAM;
      }

      pdata->bfr_next_idx = (idx + 1) % BUFFER_COUNT;

      break;
    }
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

  waveOutGetErrorText(pdata->error, pdata->error_str, ERR_STR_BUFFER_SIZE);

  return pdata->error_str;
}

static const char*
istream_get_error_op(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;

  return pdata->error_op;
}

int
janus_istream_wmm_new(janus_istream_t istream)
{
  istream->pdata = calloc(1, sizeof(struct private_data));
  istream->open = istream_open;
  istream->close = istream_close;
  istream->free = istream_free;
  istream->read = istream_read;
  istream->write = istream_write;
  istream->get_error = istream_get_error;
  istream->get_error_op = istream_get_error_op;
  strcpy(istream->name, "WMM");

  return JANUS_ERROR_NONE;
}

#endif
