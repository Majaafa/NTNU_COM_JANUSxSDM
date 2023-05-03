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
#include <janus/simple_tx.h>
#include <janus/utils/memory.h>

struct janus_simple_tx
{
  //! Verbosilty level.
  janus_uint8_t verbose;
  //! Parameter set.
  janus_pset_t pset;
  //! Output stream.
  janus_ostream_t ostream;
  //! Digital Upconverter.
  janus_duc_t duc;
  //! Tx.
  janus_tx_t tx;
};

janus_simple_tx_t
janus_simple_tx_new(janus_parameters_t params)
{
  janus_simple_tx_t simple_tx = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_simple_tx, 1);

  // Initialize verbosity level.
  simple_tx->verbose = params->verbose;

  // Configure parameter set.
  simple_tx->pset = janus_pset_new();
  janus_pset_set_id(simple_tx->pset, params->pset_id);
  if (params->pset_id != 0)
  {
    if (janus_pset_load(simple_tx->pset, params->pset_file, params->pset_id) != JANUS_ERROR_NONE)
    {
      if (simple_tx->verbose)
        fprintf(stderr, "ERROR: failed to load parameter set '%u'\n", params->pset_id);
      janus_simple_tx_free(simple_tx);
      return NULL;
    }
  }
  else
  {
    if (params->pset_center_freq != 0)
    {
      janus_pset_set_cfreq(simple_tx->pset, params->pset_center_freq);
    }
    else
    {
      if (simple_tx->verbose)
        fprintf(stderr, "ERROR: center frequency is required when no parameter set file is given\n");
      janus_simple_tx_free(simple_tx);
      return NULL;
    }

    if (params->pset_bandwidth != 0)
    {
      janus_pset_set_bwidth(simple_tx->pset, params->pset_bandwidth);
    }
    else
    {
      janus_pset_set_bwidth(simple_tx->pset, janus_pset_get_cfreq(simple_tx->pset) / 3);
      if (simple_tx->verbose)
        fprintf(stderr, "WARNING: using computed bandwidth of %u (Hz)\n", janus_pset_get_bwidth(simple_tx->pset));
    }
  }
  
  janus_pset_set_chip_len_exp(simple_tx->pset, params->chip_len_exp);
  janus_pset_set_32_chip_sequence(simple_tx->pset, params->sequence_32_chips);

  if (!janus_pset_is_valid(simple_tx->pset))
  {
    if (simple_tx->verbose)
      fprintf(stderr, "ERROR: parameter set is invalid\n");
    janus_simple_tx_free(simple_tx);
    return NULL;
  }

  if (simple_tx->verbose > 1)
    janus_pset_dump(simple_tx->pset);

  // Initialize output stream.
  simple_tx->ostream = janus_ostream_new(params->stream_driver, params->stream_driver_args);
  if (simple_tx->ostream == NULL)
  {
    if (simple_tx->verbose)
      fprintf(stderr, "ERROR: unknown output stream driver: %s\n", params->stream_driver);
    janus_simple_tx_free(simple_tx);
    return NULL;
  }
  janus_ostream_set_fs(simple_tx->ostream, params->stream_fs);
  janus_ostream_set_format(simple_tx->ostream, janus_stream_format_parse(params->stream_format));
  janus_ostream_set_channel_count(simple_tx->ostream, params->stream_channel_count);
  janus_ostream_set_channel(simple_tx->ostream, params->stream_channel);
  janus_ostream_set_multiple(simple_tx->ostream, params->stream_mul);

  // Initialize upconverter.
  if (params->stream_passband)
  {
    simple_tx->duc = janus_duc_new(params->stream_fs, simple_tx->pset->cfreq);
    janus_ostream_set_duc(simple_tx->ostream, simple_tx->duc);
  }

  // Open output stream.
  if (janus_ostream_open(simple_tx->ostream) != JANUS_ERROR_NONE)
  {
    if (simple_tx->verbose)
      fprintf(stderr, "ERROR: failed to open output stream '%s' with argument '%s': %s\n",
              params->stream_driver,
              params->stream_driver_args,
              janus_ostream_get_error(simple_tx->ostream));
    janus_simple_tx_free(simple_tx);
    return NULL;
  }

  // Dump output stream.
  if (simple_tx->verbose > 1)
    janus_ostream_dump(simple_tx->ostream);

  // Initialize tx.
  simple_tx->tx = janus_tx_new(simple_tx->pset, simple_tx->ostream, simple_tx->verbose);
  janus_tx_set_stream_amp(simple_tx->tx, params->stream_amp);
  janus_tx_set_padding(simple_tx->tx, params->pad);
  janus_tx_set_wake_up_tones(simple_tx->tx, params->wut);

  return simple_tx;
}

void
janus_simple_tx_free(janus_simple_tx_t simple_tx)
{
  if (simple_tx->tx)
    janus_tx_free(simple_tx->tx);

  if (simple_tx->duc)
    janus_duc_free(simple_tx->duc);

  if (simple_tx->ostream)
  {
    janus_ostream_close(simple_tx->ostream);
    janus_ostream_free(simple_tx->ostream);
  }

  if (simple_tx->pset)
    janus_pset_free(simple_tx->pset);

  JANUS_UTILS_MEMORY_FREE(simple_tx);
}

int
janus_simple_tx_execute(janus_simple_tx_t simple_tx, janus_packet_t packet, janus_tx_state_t state)
{
  int rv = 0;

  // Transmit.
  rv = janus_tx_execute(simple_tx->tx, packet, state);

  return rv;
}

janus_pset_t
janus_simple_tx_get_pset(janus_simple_tx_t simple_tx)
{
  return simple_tx->pset;
}

janus_ostream_t
janus_simple_tx_get_ostream(janus_simple_tx_t simple_tx)
{
  return simple_tx->ostream;
}

int
janus_simple_tx_get_ostream_status(janus_simple_tx_t simple_tx)
{
  if (simple_tx->ostream)
  {
    return janus_ostream_status(simple_tx->ostream);
  }
  
  return JANUS_ERROR_STREAM;
}

janus_duc_t
janus_simple_tx_get_duc(janus_simple_tx_t simple_tx)
{
  return simple_tx->duc;
}

janus_tx_t
janus_simple_tx_get_tx(janus_simple_tx_t simple_tx)
{
  return simple_tx->tx;
}
