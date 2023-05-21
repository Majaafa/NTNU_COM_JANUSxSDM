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

#if defined(JANUS_WITH_ALSA)

// ALSA headers.
#include <alsa/asoundlib.h>

struct private_data
{
  // Code of last error.
  int error;
  // Last error operation.
  const char* error_op;
  
  // ALSA handle.
  snd_pcm_t* fd;
  // ALSA PCM format.
  snd_pcm_format_t format;
  // ALSA PCM buffer size.
  snd_pcm_sframes_t buffer_size;
  // ALSA PCM period size.
  snd_pcm_sframes_t period_size;
};

static int
ostream_set_hw_params(janus_ostream_t ostream, snd_pcm_hw_params_t* hw_params)
{
  JANUS_OSTREAM_PDATA;

  snd_pcm_uframes_t size;
  unsigned buffer_time = 0; // Ring buffer length in us.
  unsigned period_time = 0; // Period time in us.
//   snd_pcm_uframes_t chunk_size = 1024;
  int resample = 1; // Enable alsa-lib resampling.
  int dir = 0;

  pdata->error_op = "";
  pdata->error = 0;

  // Choose all parameters.
  if ((pdata->error = snd_pcm_hw_params_any(pdata->fd, hw_params)) < 0)
  {
    pdata->error_op = "initializing parameters";
    return JANUS_ERROR_STREAM;
  }

  // Set hardware resampling.
  if ((pdata->error = snd_pcm_hw_params_set_rate_resample(pdata->fd, hw_params, resample)) < 0)
  {
    pdata->error_op = "resampling setup";
    return JANUS_ERROR_STREAM;
  }

  // Set the interleaved read/write format.
  if ((pdata->error = snd_pcm_hw_params_set_access(pdata->fd, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0)
  {
    pdata->error_op = "setting access";
    return JANUS_ERROR_STREAM;
  }

  // Set the count of channels.
  if ((pdata->error = snd_pcm_hw_params_set_channels(pdata->fd, hw_params, ostream->channel_count)) < 0)
  {
    pdata->error_op = "setting channels";
    return JANUS_ERROR_STREAM;
  }

  // Set the sample format.
  {
    pdata->format = SND_PCM_FORMAT_UNKNOWN;
    switch (ostream->format)
    {
      case JANUS_STREAM_FORMAT_DOUBLE:
        if (snd_pcm_hw_params_test_format(pdata->fd, hw_params, SND_PCM_FORMAT_FLOAT64) == 0)
          pdata->format = SND_PCM_FORMAT_FLOAT64;
        break;

      case JANUS_STREAM_FORMAT_FLOAT:
        if (snd_pcm_hw_params_test_format(pdata->fd, hw_params, SND_PCM_FORMAT_FLOAT) == 0)
          pdata->format = SND_PCM_FORMAT_FLOAT;
        break;

      case JANUS_STREAM_FORMAT_S32:
        if (snd_pcm_hw_params_test_format(pdata->fd, hw_params, SND_PCM_FORMAT_S32) == 0)
          pdata->format = SND_PCM_FORMAT_S32;
        break;

      case JANUS_STREAM_FORMAT_S24_32:
        if (snd_pcm_hw_params_test_format(pdata->fd, hw_params, SND_PCM_FORMAT_S24) == 0)
          pdata->format = SND_PCM_FORMAT_S24;
        break;
    
      case JANUS_STREAM_FORMAT_S24_3LE:
        if (snd_pcm_hw_params_test_format(pdata->fd, hw_params, SND_PCM_FORMAT_S24_3LE) == 0)
          pdata->format = SND_PCM_FORMAT_S24_3LE;
        break;

      case JANUS_STREAM_FORMAT_S16:
        if (snd_pcm_hw_params_test_format(pdata->fd, hw_params, SND_PCM_FORMAT_S16) == 0)
          pdata->format = SND_PCM_FORMAT_S16;
        break;

      case JANUS_STREAM_FORMAT_S8:
        if (snd_pcm_hw_params_test_format(pdata->fd, hw_params, SND_PCM_FORMAT_S8) == 0)
          pdata->format = SND_PCM_FORMAT_S8;
        break;

      default:
        break;
    }

    if (pdata->format == SND_PCM_FORMAT_UNKNOWN)
    {
      pdata->error_op = "configuring format";
      pdata->error = -EINVAL;
      return JANUS_ERROR_STREAM;
    }

    if ((pdata->error = snd_pcm_hw_params_set_format(pdata->fd, hw_params, pdata->format)) < 0)
    {
      pdata->error_op = "configuring format";
      return JANUS_ERROR_STREAM;
    }
  }

  // Set the stream rate.
  {
    unsigned rrate = ostream->fs;
    if ((pdata->error = snd_pcm_hw_params_set_rate_near(pdata->fd, hw_params, &rrate, 0)) < 0)
    {
      pdata->error_op = "setting sampling frequency";
      return JANUS_ERROR_STREAM;
    }
    if (rrate != ostream->fs)
    {
      pdata->error_op = "setting sampling frequency";
      pdata->error = -EINVAL;
      return JANUS_ERROR_STREAM;
    }
  }

  // Set the buffer time.
  {
    if ((pdata->error = snd_pcm_hw_params_get_buffer_time_max(hw_params, &buffer_time, 0)) < 0)
    {
      pdata->error_op = "getting maximum buffer time";
      return JANUS_ERROR_STREAM;	
    }
    if (buffer_time > 500000)
      buffer_time = 500000;
    
    period_time = buffer_time / 4;

    if ((pdata->error = snd_pcm_hw_params_set_period_time_near(pdata->fd, hw_params, &period_time, 0)) < 0)
    {
      pdata->error_op = "setting period time ";
      return JANUS_ERROR_STREAM;
    }

    if ((pdata->error = snd_pcm_hw_params_set_buffer_time_near(pdata->fd, hw_params, &buffer_time, 0)) < 0)
    {
      pdata->error_op = "setting buffer time";
      return JANUS_ERROR_STREAM;

    }

    if ((pdata->error = snd_pcm_hw_params_get_buffer_size(hw_params, &size)) < 0)
    {
      pdata->error_op = "getting buffer size";
      return JANUS_ERROR_STREAM;
    }
    pdata->buffer_size = size;
  }

  // Set the period time.
  {
    if ((pdata->error = snd_pcm_hw_params_set_period_time_near(pdata->fd, hw_params, &period_time, &dir)) < 0)
    {
      pdata->error_op = "setting period time";
      return JANUS_ERROR_STREAM;
    }
    if ((pdata->error = snd_pcm_hw_params_get_period_size(hw_params, &size, &dir)) < 0)
    {
      pdata->error_op = "getting period size";
      return JANUS_ERROR_STREAM;
    }
    pdata->period_size = size;
    janus_ostream_set_buffer_capacity(ostream, size);
  }

  // Write the hardware parameters.
  if ((pdata->error = snd_pcm_hw_params(pdata->fd, hw_params)) < 0)
  {
    pdata->error_op = "setting hardware parameters";
    return JANUS_ERROR_STREAM;
  }

  return 0;
}

static int
ostream_set_sw_params(janus_ostream_t ostream, snd_pcm_sw_params_t* sw_params)
{
  JANUS_OSTREAM_PDATA;

  pdata->error_op = "";
  pdata->error = 0;

  // Get the current software params.
  if ((pdata->error = snd_pcm_sw_params_current(pdata->fd, sw_params)) < 0)
  {
    pdata->error_op = "getting current software parameters";
    return JANUS_ERROR_STREAM;
  }

  // Start the transfer when the buffer is almost full: (buffer_size / avail_min) * avail_min
  if ((pdata->error = snd_pcm_sw_params_set_start_threshold(pdata->fd, sw_params, (pdata->buffer_size / pdata->period_size) * pdata->period_size)) < 0)
  {
    pdata->error_op = "setting start threshold mode";
    return JANUS_ERROR_STREAM;
  }

  // Allow the transfer when at least period_size samples can be processed.
  if ((pdata->error = snd_pcm_sw_params_set_avail_min(pdata->fd, sw_params, pdata->period_size)) < 0)
  {
    pdata->error_op = "setting avail min";
    return JANUS_ERROR_STREAM;
  }

  // Write the software parameters.
  if ((pdata->error = snd_pcm_sw_params(pdata->fd, sw_params)) < 0)
  {
    pdata->error_op = "setting software parameters";
    return JANUS_ERROR_STREAM;
  }

  return 0;
}

static int
ostream_open(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  snd_pcm_hw_params_t* hw_params = NULL;
  snd_pcm_sw_params_t* sw_params = NULL;

  pdata->error_op = "";
  pdata->error = 0;

  // Open device.
  if ((pdata->error = snd_pcm_open(&pdata->fd, ostream->args, SND_PCM_STREAM_PLAYBACK, 0)) < 0)
  {
    pdata->error_op = "opening device";
    return JANUS_ERROR_STREAM;
  }

  // Allocating hardware parameters structure.
  if ((pdata->error = snd_pcm_hw_params_malloc(&hw_params)) < 0)
  {
    pdata->error_op = "allocating hardware parameters";
    goto cleanup;
  }

  memset(hw_params, 0, snd_pcm_hw_params_sizeof());

  // Allocating software parameters structure.
  if ((pdata->error = snd_pcm_sw_params_malloc(&sw_params)) < 0)
  {
    pdata->error_op = "allocating software parameters";
    goto cleanup;
  }

  memset(sw_params, 0, snd_pcm_sw_params_sizeof());

  // Setting hardware parameters structure.
  if ((pdata->error = ostream_set_hw_params(ostream, hw_params)) < 0)
  {
    goto cleanup;
  }

  // Setting software parameters structure.
  if ((pdata->error = ostream_set_sw_params(ostream, sw_params)) < 0)
  {
    goto cleanup;
  }

  // Preparing PCM for use.
  if ((pdata->error = snd_pcm_prepare(pdata->fd)) < 0)
  {
    pdata->error_op = "preparing interface";
    goto cleanup;
  }

  snd_pcm_hw_params_free(hw_params);
  snd_pcm_sw_params_free(sw_params);
  return JANUS_ERROR_NONE;

 cleanup:
  snd_pcm_hw_params_free(hw_params);
  snd_pcm_sw_params_free(sw_params);
  return JANUS_ERROR_STREAM;
}

static int
ostream_close(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  if (pdata->fd)
  {
    snd_pcm_drain(pdata->fd);
    snd_pcm_hw_free(pdata->fd);
    snd_pcm_close(pdata->fd);
    pdata->fd = NULL;
    snd_config_update_free_global();
  }

  return JANUS_ERROR_NONE;
}

static int
ostream_reset(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;
  
  if (pdata->fd)
  {
    snd_pcm_drain(pdata->fd);
    
    if ((pdata->error = snd_pcm_prepare(pdata->fd)) < 0)
    {
      pdata->error_op = "recovering from underrun, prepare failed";
      return JANUS_ERROR_STREAM;
    }
  }

  return JANUS_ERROR_NONE;
}

static int
ostream_status(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;
  
  if (pdata->fd)
  {
    switch (snd_pcm_state(pdata->fd))
    {
      case SND_PCM_STATE_OPEN:
        //printf("Open\n");
        return JANUS_ERROR_STREAM;
      case SND_PCM_STATE_SETUP:
        //printf("Setup installed\n");
        return JANUS_ERROR_STREAM;
      case SND_PCM_STATE_PREPARED:
        //printf("Ready to start\n");
        return JANUS_ERROR_NONE;
      case SND_PCM_STATE_RUNNING:
        //printf("Running\n");
        return JANUS_ERROR_NONE;
      case SND_PCM_STATE_XRUN:
        //printf("Stopped: underrun (playback) or overrun (capture) detected\n");
        return JANUS_ERROR_UNDERRUN;
      case SND_PCM_STATE_DRAINING:
        //printf("Draining: running (playback) or stopped (capture)\n");
        return JANUS_ERROR_NONE;
      case SND_PCM_STATE_PAUSED:
        //printf("Paused\n");
        return JANUS_ERROR_STREAM;
      case SND_PCM_STATE_SUSPENDED:
        //printf("Hardware is suspended\n");
        return JANUS_ERROR_STREAM;
      case SND_PCM_STATE_DISCONNECTED:
        //printf("Hardware is disconnected\n");
        return JANUS_ERROR_STREAM;
    }
  }
  
  return JANUS_ERROR_OTHER;
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
ostream_recovery(janus_ostream_t ostream, int err)
{
  JANUS_OSTREAM_PDATA;

  if (err == -EPIPE)
  {
    // Underrun occurred.
    if ((pdata->error = snd_pcm_prepare(pdata->fd)) < 0)
    {
      pdata->error_op = "recovering from underrun, prepare failed";
      return JANUS_ERROR_STREAM;
    }

    return 0;
  }

  if (err == -ESTRPIPE)
  {
    // Suspend event occurred.
    while ((err = snd_pcm_resume(pdata->fd)) == -EAGAIN)
    {
      sleep(1); // Wait until the suspend flag is released.
    }

    if (err < 0)
    {
      if ((pdata->error = snd_pcm_prepare(pdata->fd)) < 0)
      {
        pdata->error_op = "recovering from suspend, prepare failed";
        return JANUS_ERROR_STREAM;
      }
    }

    return 0;
  }

  return err;
}

static int
ostream_write(janus_ostream_t ostream, void* frame, unsigned frame_count)
{
  JANUS_OSTREAM_PDATA;

  int rv = 0;
  char* ptr = frame;
  int remaining = frame_count;
  
#if 0
  {
    switch (snd_pcm_state(pdata->fd))
    {
      case SND_PCM_STATE_OPEN:
        printf("Open\n");
        break;
      case SND_PCM_STATE_SETUP:
        printf("Setup installed\n");
        break;
      case SND_PCM_STATE_PREPARED:
        printf("Ready to start\n");
        break;
      case SND_PCM_STATE_RUNNING:
        printf("Running\n");
        break;
      case SND_PCM_STATE_XRUN:
        printf("Stopped: underrun (playback) or overrun (capture) detected\n");
        if ((pdata->error = snd_pcm_prepare(pdata->fd)) < 0)
        {
          pdata->error_op = "recovering from underrun, prepare failed";
        }
        break;
      case SND_PCM_STATE_DRAINING:
        printf("Draining: running (playback) or stopped (capture)\n");
        break;
      case SND_PCM_STATE_PAUSED:
        printf("Paused\n");
        break;
      case SND_PCM_STATE_SUSPENDED:
        printf("Hardware is suspended\n");
        break;
      case SND_PCM_STATE_DISCONNECTED:
        printf("Hardware is disconnected\n");
        break;
    }
  }
#endif

  if (frame_count < pdata->period_size)
  {
    snd_pcm_format_set_silence(pdata->format, (char*)frame + frame_count * janus_ostream_get_sample_size(ostream) * ostream->channel_count, (pdata->period_size - frame_count) * ostream->channel_count);
    remaining = pdata->period_size;
  }
  
  while (remaining > 0)
  {
    rv = snd_pcm_writei(pdata->fd, ptr, remaining);

    if (rv == -EAGAIN)
      continue;

    if (rv < 0)
    {
      if ((pdata->error = ostream_recovery(ostream, rv)) < 0)
      {
        return JANUS_ERROR_STREAM;
      }
      break; // Skip one period.
    }

    ptr += rv * ostream->channel_count;
    remaining -= rv;
  }
  
  return rv;
}

static const char*
ostream_get_error(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;
  return snd_strerror(pdata->error);
}

static const char*
ostream_get_error_op(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;
  return pdata->error_op;
}

int
janus_ostream_alsa_new(janus_ostream_t ostream)
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
  strcpy(ostream->name, "ALSA");

  return JANUS_ERROR_NONE;
}

#endif
