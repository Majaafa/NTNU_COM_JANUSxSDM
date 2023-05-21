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

#ifndef JANUS_RX_STATE_H_INCLUDED_
#define JANUS_RX_STATE_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>
#include <janus/parameters.h>

/* Parameters for Colored Bit Probabilities (CBP) Terminal Dump */
#define CBP_HIGH2MEDIUM_MAX  0.45
#define CBP_MEDIUM2LOW_MAX   0.50

#define RED   "\x1B[31m"
#define YLW   "\x1B[33m"
#define GRN   "\x1B[32m"
#define RESET "\x1B[0m"

#define CBP_HIGH   GRN
#define CBP_MEDIUM YLW
#define CBP_LOW    RED

typedef struct janus_rx_state* janus_rx_state_t;

struct janus_rx_state
{
  janus_uint16_t pset_id;
  char* pset_name;
  unsigned cfreq;
  unsigned bwidth;
  unsigned chip_frq;
  janus_real_t chip_dur;

  janus_real_t after;
  janus_real_t gamma;
  janus_real_t speed;

  janus_uint8_t rssi;
  janus_real_t  snr;

  // Bit probabilities in the range 0 < p < 1.
  janus_real_t* bit_prob;
  // Bit probabilities size (if -1 disable bit_prob).
  int bit_prob_size;
  // Colored Bit Probabilities: enable/disable
  janus_uint8_t colored_bit_prob;
  // Colored Bit Probabilities: High to Medium Probability Threshold for a '0'
  janus_real_t cbp_high2medium_zero;
  // Colored Bit Probabilities: High to Medium Probability Threshold for a '1'
  janus_real_t cbp_high2medium_one;
  // Colored Bit Probabilities: Medium to Low Probability Threshold for a '0'
  janus_real_t cbp_medium2low_zero;
  // Colored Bit Probabilities: Medium to Low Probability Threshold for a '0'
  janus_real_t cbp_medium2low_one;
};

JANUS_EXPORT janus_rx_state_t
janus_rx_state_new(janus_parameters_t params);

JANUS_EXPORT void
janus_rx_state_free(janus_rx_state_t rx_state);

JANUS_EXPORT void
janus_rx_state_dump(const janus_rx_state_t rx_state);

#endif
