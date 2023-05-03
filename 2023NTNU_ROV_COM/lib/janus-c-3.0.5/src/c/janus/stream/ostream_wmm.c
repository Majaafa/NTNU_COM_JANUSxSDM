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
// Authors: Ricardo Martins, Luigi Elia D'Amaro, Giovanni Zappa           *
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

#define BUFFER_COUNT        16
#define ERR_STR_BUFFER_SIZE 128

struct private_data
{
  // Code of last error.
  int error;
  // Last error operation.
  const char* error_op;

  // Wave out handle.
  HWAVEOUT fd;
  // Error string buffer.
  char error_str[ERR_STR_BUFFER_SIZE];
  // Buffers.
  WAVEHDR** bfr;
 
  // Current buffer index.
  int current_buffer;
  // Index in current buffer.
  int n_bytes;

  // Semphore.
  HANDLE sem;
};

static WAVEHDR*
ostream_new_buffer(janus_ostream_t ostream, unsigned size)
{
  JANUS_OSTREAM_PDATA;

  WAVEHDR* hdr = (WAVEHDR*)calloc(1, sizeof(WAVEHDR));
  MMRESULT rv;

  hdr->dwLoops = 0;
  hdr->dwFlags = 0;
  hdr->lpData = (LPSTR)malloc(size);
  hdr->dwBufferLength = size;
  pdata->n_bytes = 0;
  rv = waveOutPrepareHeader(pdata->fd, hdr, sizeof(WAVEHDR));
  if (rv != MMSYSERR_NOERROR)
  {
    free(hdr->lpData);
    free(hdr);
    return 0;
  }

  return hdr;
}

static void CALLBACK
wave_callback(HWAVEOUT h_wave, UINT msg, DWORD user, DWORD dw1, DWORD dw2)
{
  if (msg == WOM_DONE)
  {
    ReleaseSemaphore((HANDLE)user, 1, NULL);
  }
}

static int
ostream_open(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  MMRESULT rv;
  WAVEFORMATEX fmt;

  fmt.wFormatTag = WAVE_FORMAT_PCM;
  fmt.nChannels = ostream->channel_count;
  fmt.nSamplesPerSec = ostream->fs;
  fmt.wBitsPerSample = janus_stream_format_get_sample_size_bits(ostream->format);
  fmt.nBlockAlign = ostream->frame_size;
  fmt.nAvgBytesPerSec = fmt.nBlockAlign * fmt.nSamplesPerSec;
  fmt.cbSize = 0;

  pdata->sem = CreateSemaphore(NULL, 0, BUFFER_COUNT, NULL);
  pdata->current_buffer = 0;

  rv = waveOutOpen(&pdata->fd,
                   atoi(ostream->args),
                   &fmt,
                   (DWORD_PTR)wave_callback,
                   (DWORD_PTR)pdata->sem,
                   CALLBACK_FUNCTION);

  if (rv != MMSYSERR_NOERROR)
  {
    pdata->error = rv;
    pdata->error_op = "opening output device";
    return JANUS_ERROR_STREAM;
  }

  // Allocate internal buffers.
  pdata->bfr = (WAVEHDR**)calloc(BUFFER_COUNT, sizeof(WAVEHDR*));

  {
    unsigned frame_count = janus_ostream_get_buffer_capacity(ostream);
    int i;
    for (i = 0; i < BUFFER_COUNT; ++i)
    {
      pdata->bfr[i] = ostream_new_buffer(ostream, frame_count * fmt.nBlockAlign);
      if (pdata->bfr[i] == 0)
      {
        pdata->error = rv;
        pdata->error_op = "preparing waveform data block";
        return JANUS_ERROR_STREAM;
      }
    }
  }

  return JANUS_ERROR_NONE;
}

static int
ostream_close(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  MMRESULT rv;
  int i;

  do
  {
    rv = waveOutClose(pdata->fd);
  }
  while (rv == WAVERR_STILLPLAYING);

  for (i = 0; i < BUFFER_COUNT; ++i)
  {
    free(pdata->bfr[i]->lpData);
    free(pdata->bfr[i]);
  }
  free(pdata->bfr);

  CloseHandle(pdata->sem);

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
ostream_wave_buffer_write(janus_ostream_t ostream, WAVEHDR* ptr, void* framedata, int n_bytes, int* bytes_written)
{
  JANUS_OSTREAM_PDATA;

  MMRESULT rv;  

  // ASSERT((DWORD)n_bytes != ptr->dwBufferLength);
  *bytes_written = min((int)ptr->dwBufferLength - pdata->n_bytes, n_bytes);
  CopyMemory((ptr->lpData + pdata->n_bytes), framedata, *bytes_written);
  pdata->n_bytes += *bytes_written;
    
  if (pdata->n_bytes == (int)ptr->dwBufferLength) 
  {
    /*  Write it! */
    rv = waveOutWrite(pdata->fd, ptr, sizeof(WAVEHDR));
    if (rv != MMSYSERR_NOERROR)
    {
      pdata->error = rv;
      pdata->error_op = "sending data block to the output device";
      return JANUS_ERROR_STREAM;
    }
    rv = pdata->n_bytes;
    pdata->n_bytes = 0;
    return rv;
  }
  return 0;
}

static int
ostream_write(janus_ostream_t ostream, void* frame, unsigned frame_count)
{
  JANUS_OSTREAM_PDATA;

  int rv;
  WAVEHDR* ptr = NULL;

  while (frame_count != 0) 
  {
    int n_written;
    
    ptr = pdata->bfr[pdata->current_buffer];

    /*  Get a buffer if necessary. */
    while ((ptr->dwFlags & WHDR_INQUEUE) != 0)
    {
      WaitForSingleObject(pdata->sem, INFINITE);
    }

    if ((rv = ostream_wave_buffer_write(ostream, ptr, frame, frame_count * ostream->frame_size, &n_written)) > 0)
    {
      LPSTR pframe = (LPSTR)frame;
      pdata->current_buffer = (pdata->current_buffer + 1) % BUFFER_COUNT;
      frame_count -= n_written / ostream->frame_size;
      pframe += n_written;
      frame = pframe;
    } 
    else 
    {
      // ASSERT(n_written == frame_count);
      break;
    }
  }

  return rv;
}

static const char*
ostream_get_error(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  waveOutGetErrorText(pdata->error, pdata->error_str, ERR_STR_BUFFER_SIZE);

  return pdata->error_str;
}

static const char*
ostream_get_error_op(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  return pdata->error_op;
}

int
janus_ostream_wmm_new(janus_ostream_t ostream)
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
  strcpy(ostream->name, "WMM");

  return JANUS_ERROR_NONE;
}

#endif
