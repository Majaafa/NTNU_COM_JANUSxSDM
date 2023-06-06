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
#include <stdlib.h>
#include <stdio.h>

// JANUS headers.
#include <janus/janus.h>
#include <janus/defaults.h>
#include <janus/simple_rx.h>
#include <janus/utils/memory.h>

#include <unistd.h> //WM for pid 
#include <sys/types.h> //WM for pid 

int justonce = 0;

struct janus_simple_rx
{
  //! Verbosilty level.
  janus_uint8_t verbose;
  //! Parameter set.
  janus_pset_t pset;
  //! Input stream.
  janus_istream_t istream;
  //! Digital Downconverter.
  janus_ddc_t ddc;
  //! Rx.
  janus_rx_t rx;
};

janus_simple_rx_t
janus_simple_rx_new(janus_parameters_t params)
{
  janus_simple_rx_t simple_rx = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_simple_rx, 1);
  

  //pid_t pidtoexec = getpid();
  //fprintf(stderr, "\n%d:PIDKLARJANUS\n", pidtoexec);

  
  // Initialize verbosity level
  simple_rx->verbose = params->verbose;
  
  // Configure parameter set.
  simple_rx->pset = janus_pset_new();
  janus_pset_set_id(simple_rx->pset, params->pset_id);

  if (params->pset_id != 0)
  {
    int rv = janus_pset_load(simple_rx->pset, params->pset_file, params->pset_id);

    if (rv != JANUS_ERROR_NONE)
    {
      if (simple_rx->verbose) {
	switch (rv) {
	  case JANUS_ERROR_FILE :
	    fprintf(stderr, "ERROR: failed to load file '%s'\n", params->pset_file);  
	    break;
	    
	  case JANUS_ERROR_NOT_FOUND:
	  default:
	    fprintf(stderr, "ERROR: failed to load parameter set '%u' from '%s'\n", params->pset_id,  params->pset_file);  
	}
      }
	
      janus_simple_rx_free(simple_rx);
      return NULL;
    }
  }
  else
  {
    if (params->pset_center_freq != 0)
    {
      janus_pset_set_cfreq(simple_rx->pset, params->pset_center_freq);
    }
    else
    {
      if (simple_rx->verbose)
        fprintf(stderr, "ERROR: center frequency is required when no parameter set file is given\n");
      janus_simple_rx_free(simple_rx);
      return NULL;
    }

    if (params->pset_bandwidth != 0)
    {
      janus_pset_set_bwidth(simple_rx->pset, params->pset_bandwidth);
    }
    else
    {
      janus_pset_set_bwidth(simple_rx->pset, janus_pset_get_cfreq(simple_rx->pset) / 3);
      if (simple_rx->verbose)
        fprintf(stderr, "WARNING: using computed bandwidth of %u (Hz)\n", janus_pset_get_bwidth(simple_rx->pset));
    }
  }

  janus_pset_set_chip_len_exp(simple_rx->pset, params->chip_len_exp);
  janus_pset_set_32_chip_sequence(simple_rx->pset, params->sequence_32_chips);

  if (!janus_pset_is_valid(simple_rx->pset))
  {
    if (simple_rx->verbose)
      fprintf(stderr, "ERROR: parameter set is invalid\n");
    janus_simple_rx_free(simple_rx);
    return NULL;
  }

  if (simple_rx->verbose > 1)
    janus_pset_dump(simple_rx->pset);

  // Initialize input stream.
  simple_rx->istream = janus_istream_new(params->stream_driver, params->stream_driver_args);
  if (simple_rx->istream == NULL)
  {
    if (simple_rx->verbose)
      fprintf(stderr, "error: unknown input stream driver: %s\n", params->stream_driver);
    janus_simple_rx_free(simple_rx);
    return NULL;
  }
  janus_istream_set_fs(simple_rx->istream, params->stream_fs);
  janus_istream_set_format(simple_rx->istream, janus_stream_format_parse(params->stream_format));
  janus_istream_set_channel_count(simple_rx->istream, params->stream_channel_count);
  janus_istream_set_channel(simple_rx->istream, params->stream_channel);

  // Initialize downconverter.
  if (params->stream_passband)
  {
    simple_rx->ddc = janus_ddc_new(params->stream_fs, simple_rx->pset->cfreq, simple_rx->pset->ubwidth);
    janus_istream_set_ddc(simple_rx->istream, simple_rx->ddc);
  }

  // Open input stream.
  if (janus_istream_open(simple_rx->istream) != JANUS_ERROR_NONE)
  {
    if (simple_rx->verbose)
      fprintf(stderr, "ERROR: failed to open input stream '%s' with argument '%s': %s\n",
              params->stream_driver,
              params->stream_driver_args,
              janus_istream_get_error(simple_rx->istream));
    janus_simple_rx_free(simple_rx);
    return NULL;
  }

  // Dump input stream.
  if (simple_rx->verbose > 1)
    janus_istream_dump(simple_rx->istream);
  
  // Initialize rx.
  simple_rx->rx = janus_rx_new(simple_rx->pset, params, simple_rx->istream, simple_rx->verbose);

  return simple_rx;
}

void
janus_simple_rx_free(janus_simple_rx_t simple_rx)
{
  if (simple_rx->rx)
    janus_rx_free(simple_rx->rx);

  if (simple_rx->ddc)
    janus_ddc_free(simple_rx->ddc);

  if (simple_rx->istream)
  {
    janus_istream_close(simple_rx->istream);
    janus_istream_free(simple_rx->istream);
  }

  if (simple_rx->pset)
    janus_pset_free(simple_rx->pset);

  JANUS_UTILS_MEMORY_FREE(simple_rx);
}

int
janus_simple_rx_execute(janus_simple_rx_t simple_rx, janus_packet_t packet, janus_rx_state_t state)
{
  int rv = 0;

  // Receive.
  rv = janus_rx_execute(simple_rx->rx, packet, state);

  return rv;
}

janus_pset_t
janus_simple_rx_get_pset(janus_simple_rx_t simple_rx)
{
  return simple_rx->pset;
}

janus_istream_t
janus_simple_rx_get_istream(janus_simple_rx_t simple_rx)
{
  return simple_rx->istream;
}

janus_ddc_t
janus_simple_rx_get_ddc(janus_simple_rx_t simple_rx)
{
  return simple_rx->ddc;
}

janus_rx_t
janus_simple_rx_get_rx(janus_simple_rx_t simple_rx)
{
  return simple_rx->rx;
}

int
janus_simple_rx_has_detected(janus_simple_rx_t simple_rx)
{
  return janus_rx_has_detected(simple_rx->rx);
}

inline int
janus_simple_rx_is_decoding(janus_simple_rx_t simple_rx)
{
  return janus_rx_is_decoding(simple_rx->rx);
}

janus_real_t
janus_simple_rx_get_first_detection_time(janus_simple_rx_t simple_rx)
{
  return janus_rx_get_first_detection_time(simple_rx->rx);
}

inline janus_real_t
janus_simple_rx_get_detection_time(janus_simple_rx_t simple_rx)
{
  return janus_rx_get_detection_time(simple_rx->rx);
}

inline void
janus_simple_rx_get_frequencies(janus_simple_rx_t simple_rx, unsigned* freq_vector_size, janus_real_t** freq_vector)
{
  janus_rx_get_frequencies(simple_rx->rx, freq_vector_size, freq_vector);
}

inline int
janus_simple_rx_get_channel_spectrogram(janus_simple_rx_t simple_rx, janus_real_t** spectrogram, unsigned* valid, janus_real_t* last_time, janus_real_t* step_time)
{
  return janus_rx_get_channel_spectrogram(simple_rx->rx, spectrogram, valid, last_time, step_time);
}
