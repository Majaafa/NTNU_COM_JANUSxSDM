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
// Authors: Giovanni Zappa, Luigi Elia D'Amaro                            *
//*************************************************************************

// ISO C headers.
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// JANUS headers.
#include <janus/defaults.h>
#include <janus/parameters.h>
#include <janus/utils/memory.h>

janus_parameters_t
janus_parameters_new(void)
{
  janus_parameters_t params = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_parameters, 1);

  params->verbose = 0;

  params->pset_id = 0;
  params->pset_file = NULL;
  params->pset_center_freq = 0;
  params->pset_bandwidth = 0;
  params->chip_len_exp = 0;
  params->sequence_32_chips = JANUS_32_CHIP_SEQUENCE;

  // Stream parameters.
  params->stream_driver = "wav";
  params->stream_driver_args = "janus.wav";
  params->stream_fs = 44100;
  params->stream_format = "S16";
  params->stream_passband = 1;
  params->stream_amp = JANUS_REAL_CONST(0.95);
  params->stream_mul = 1;

  // Tx parameters.
  params->pad = 1;
  params->wut = 0;

  // Rx parameters.
  params->doppler_correction = 1;
  params->doppler_max_speed = JANUS_REAL_CONST(5.0);
  params->compute_channel_spectrogram = 1;
  params->detection_threshold = JANUS_REAL_CONST(2.5);
  params->colored_bit_prob = 0;
  params->cbp_high2medium  = JANUS_REAL_CONST(0.2);
  params->cbp_medium2low   = JANUS_REAL_CONST(0.35);

  return params;
}

void
janus_parameters_free(janus_parameters_t params)
{
  JANUS_UTILS_MEMORY_FREE(params);
}
