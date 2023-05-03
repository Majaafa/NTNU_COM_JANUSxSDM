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
istream_set_hw_params(janus_istream_t istream, snd_pcm_hw_params_t* hw_params)
{
  JANUS_ISTREAM_PDATA;

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
  if ((pdata->error = snd_pcm_hw_params_set_channels(pdata->fd, hw_params, istream->channel_count)) < 0)
  {
    pdata->error_op = "setting channels";
    return JANUS_ERROR_STREAM;
  }

  // Set the sample format.
  {
    pdata->format = SND_PCM_FORMAT_UNKNOWN;
    switch (istream->format)
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
    unsigned rrate = istream->fs;
    if ((pdata->error = snd_pcm_hw_params_set_rate_near(pdata->fd, hw_params, &rrate, 0)) < 0)
    {
      pdata->error_op = "setting sampling frequency";
      return JANUS_ERROR_STREAM;
    }
    if (rrate != istream->fs)
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
      pdata->error_op = "setting  buffer time";
      return JANUS_ERROR_STREAM;	
    }
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
  }

  // Write the hardware parameters.
  if ((pdata->error = snd_pcm_hw_params(pdata->fd, hw_params)) < 0)
  {
    pdata->error_op = "setting hardware parameters";
    return JANUS_ERROR_STREAM;
  }

#if 1
  {
    int err;
    snd_output_t *log;

    err = snd_output_stdio_attach(&log, stderr, 0);
    assert(err >= 0);

    fprintf(stderr, "HW params of device \"%s\":\n", snd_pcm_name(pdata->fd));
    fprintf(stderr, "--------------------\n");
    snd_pcm_hw_params_dump(hw_params, log);
    fprintf(stderr, "--------------------\n");
    
    fflush(stdout);
    snd_output_close (log);
  }
#endif

  return 0;
}

static int
istream_set_sw_params(janus_istream_t istream, snd_pcm_sw_params_t* sw_params)
{
  JANUS_ISTREAM_PDATA;

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
istream_open(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;

  snd_pcm_hw_params_t* hw_params = NULL;
  snd_pcm_sw_params_t* sw_params = NULL;

  pdata->error_op = "";
  pdata->error = 0;

  // Open device.
  if ((pdata->error = snd_pcm_open(&pdata->fd, istream->args, SND_PCM_STREAM_CAPTURE, 0)) < 0)
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
  if ((pdata->error = istream_set_hw_params(istream, hw_params)) < 0)
  {
    goto cleanup;
  }

  // Setting software parameters structure.
  if ((pdata->error = istream_set_sw_params(istream, sw_params)) < 0)
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
istream_close(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;

  if (pdata->fd)
  {
    snd_pcm_drain(pdata->fd);
    snd_pcm_close(pdata->fd);
    pdata->fd = NULL;
    snd_config_update_free_global();
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

  int rv = snd_pcm_readi(pdata->fd, frames, frame_count);
  if (rv <= 0)
  {
    pdata->error = rv;
    // Trying recovery.
    if ((pdata->error = snd_pcm_prepare(pdata->fd)) < 0)
    {
      pdata->error_op = "Attempting to recover.";
      return JANUS_ERROR_OVERRUN;
    }

    return 0;
  }

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
  return snd_strerror(pdata->error);
}

static const char*
istream_get_error_op(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;
  return pdata->error_op;
}

int
janus_istream_alsa_new(janus_istream_t istream)
{
  istream->pdata = calloc(1, sizeof(struct private_data));
  istream->open = istream_open;
  istream->close = istream_close;
  istream->free = istream_free;
  istream->read = istream_read;
  istream->write = istream_write;
  istream->get_error = istream_get_error;
  istream->get_error_op = istream_get_error_op;
  strcpy(istream->name, "ALSA");

  return JANUS_ERROR_NONE;
}

#endif
