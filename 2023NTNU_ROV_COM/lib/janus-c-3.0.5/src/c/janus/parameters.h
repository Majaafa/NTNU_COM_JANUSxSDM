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

#ifndef JANUS_PARAMETERS_H_INCLUDED_
#define JANUS_PARAMETERS_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>

typedef struct janus_parameters* janus_parameters_t;

struct janus_parameters
{
  //! Verbose: vebosity level.
  janus_uint8_t verbose;

  //! Parameter Set: Identifier.
  unsigned pset_id;
  //! Parameter Set: File.
  const char* pset_file;
  //! Parameter Set: Center Frequency.
  unsigned pset_center_freq;
  //! Parameter Set: Bandwidth.
  unsigned pset_bandwidth;

  //! Chip length dyadic exponent.
  janus_uint16_t chip_len_exp;
  //! Initial fixed sequence of 32 chips.
  janus_uint32_t sequence_32_chips;

  //! Stream parameters.

  //! Stream: Driver (null, raw, wav, fifo, alsa, pulse, wmm).
  const char* stream_driver;
  //! Stream: Driver Arguments.
  const char* stream_driver_args;
  //! Stream: Sampling Frequency (Hz).
  unsigned stream_fs;
  //! Stream: Format (S8, S10, S12, S14, S16, S24, S24_32, S32, FLOAT, DOUBLE).
  const char* stream_format;
  //! Number of stream channels.
  unsigned stream_channel_count;
  //! Signal channel.
  unsigned stream_channel;
  //! Stream: Passband or Baseband Signal.
  janus_uint8_t stream_passband;
  //! Stream: amplitude factor.
  janus_real_t stream_amp;
  //! Stream: force number of output samples to be a multiple of a given number.
  janus_uint8_t stream_mul;

  //! Tx parameters.

  //! Padding: enabled/disabled.
  janus_uint8_t pad;
  //! Wake Up Tones: enabled/disabled
  janus_uint8_t wut;

  //! Rx parameters.

  //! Doppler correction: enabled/disabled.
  janus_uint8_t doppler_correction;
  //! Doppler correction: maximum speed [m/s].
  janus_real_t doppler_max_speed;
  //! Channel Spectrogram Computation: enabled/disabled.
  janus_uint8_t compute_channel_spectrogram;
  //! Detection threshold.
  janus_real_t detection_threshold;
  //! Colored Bit Probabilities: enable/disable
  janus_uint8_t colored_bit_prob;
  //! Colored Bit Probabilities: High to Medium Probability Threshold
  janus_real_t cbp_high2medium;
  //! Colored Bit Probabilities: Medium to Low Probability Threshold
  janus_real_t cbp_medium2low;
  //! Return after demodulation: enabled/disabled.
  janus_uint8_t rx_once;
  //! Assume, the provided signal is already synchronized on the first chip
  janus_uint8_t skip_detection;
  //! Signal offset of the first symbol to be demodulated
  janus_uint32_t detected_offset;
  //! Doppler value
  janus_real_t detected_doppler;
};

JANUS_EXPORT janus_parameters_t
janus_parameters_new(void);

JANUS_EXPORT void
janus_parameters_free(janus_parameters_t params);

#endif
