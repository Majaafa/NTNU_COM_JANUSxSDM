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
#include <janus/dump.h>
#include <janus/rx_state.h>
#include <janus/utils/memory.h>

janus_rx_state_t
janus_rx_state_new(janus_parameters_t params)
{
  janus_rx_state_t rx_state = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_rx_state, 1);

  if (0 == (params->verbose > 1))
    rx_state->bit_prob_size = -1;

  /* Check the Sanity of the CLI user values */
  if ((params->cbp_high2medium > 0.0) && \
      (params->cbp_high2medium < CBP_HIGH2MEDIUM_MAX) && \
      (params->cbp_medium2low  > 0.0) && \
      (params->cbp_medium2low  < CBP_MEDIUM2LOW_MAX) && \
      (params->cbp_high2medium < params->cbp_medium2low))
  {
    rx_state->colored_bit_prob = params->colored_bit_prob;

    rx_state->cbp_high2medium_zero = params->cbp_high2medium;
    rx_state->cbp_high2medium_one  = 1.0 - params->cbp_high2medium;
    rx_state->cbp_medium2low_zero  = params->cbp_medium2low;
    rx_state->cbp_medium2low_one   = 1.0 - params->cbp_medium2low;

    return rx_state;
  }
  else
  {
    return NULL;
  }
}

void
janus_rx_state_free(janus_rx_state_t rx_state)
{
  if (rx_state->bit_prob_size != -1)
    JANUS_UTILS_MEMORY_FREE(rx_state->bit_prob);

  JANUS_UTILS_MEMORY_FREE(rx_state);
}

void
janus_rx_state_dump(const janus_rx_state_t rx_state)
{
  JANUS_DUMP("State", "Parameter Set Id", "%u", rx_state->pset_id);
  JANUS_DUMP("State", "Parameter Set Name", "%s", rx_state->pset_name);
  JANUS_DUMP("State", "Center Frequency (Hz)", "%u", rx_state->cfreq);
  JANUS_DUMP("State", "Available Bandwidth (Hz)", "%u", rx_state->bwidth);
  JANUS_DUMP("State", "Chip Frequency (Hz)", "%u", rx_state->chip_frq);
  JANUS_DUMP("State", "Chip Duration (s)", "%0.6f", rx_state->chip_dur);

  JANUS_DUMP("State", "After (s)", "%0.6f", rx_state->after);
  JANUS_DUMP("State", "Gamma", "%0.6f", rx_state->gamma);
  JANUS_DUMP("State", "Speed (m/s)", "%0.6f", rx_state->speed);

  JANUS_DUMP("State", "RSSI", "%u", rx_state->rssi);
  JANUS_DUMP("State", "SNR", "%0.3f", rx_state->snr);

  if (rx_state->bit_prob_size != -1)
  {
    int i;

    if (1 == rx_state->colored_bit_prob)
    {
      JANUS_DUMP("State", "Bit probabilities (percent)", "%s", "");
      for (i = 0; i < rx_state->bit_prob_size; ++i)
      {
        if (0 == (i%16) )
        {
          /* Print Table Header and 1st column */
          if (0 == i)
          {
            /* Print Table Header */
            fprintf(stderr, "           01  02  03  04  05  06  07  08"\
                            "  09  10  11  12  13  14  15  16");
            fprintf(stderr, "\n%03d..%03d: ", i + 1, i + 16);
          }
          else
          {
            fprintf(stderr, "\n%03d..%03d: ", i + 1, i + 16);
          }
        }
        /* Classify Bit Probability */
        if ((rx_state->bit_prob[i] >  rx_state->cbp_medium2low_zero) && \
            (rx_state->bit_prob[i] <  rx_state->cbp_medium2low_one))
        {
          /* Low Bit Probability */
          fprintf(stderr, CBP_LOW "%03d " RESET,\
                  (int) (rx_state->bit_prob[i] * 100.0));
        }
        else if  ((rx_state->bit_prob[i] >  rx_state->cbp_high2medium_zero) && \
                  (rx_state->bit_prob[i] <  rx_state->cbp_high2medium_one))
        {
          /* Medium Bit Probability */
          fprintf(stderr, CBP_MEDIUM "%03d " RESET,\
                  (int) (rx_state->bit_prob[i]));
        }
        else
        {
          /* High Bit Probability */
          fprintf(stderr, CBP_HIGH "%03d " RESET,\
                  (int) (rx_state->bit_prob[i] * 100.0));
        }
      }
      fprintf(stderr, "\n");
    }
    else
    {
      char* string_bit_prob = JANUS_UTILS_MEMORY_ALLOCA(char, rx_state->bit_prob_size * 6 + 1);
      string_bit_prob[0] = '\0';
      for (i = 0; i < rx_state->bit_prob_size; ++i)
      {
        APPEND_FORMATTED(string_bit_prob, "%.3f ", rx_state->bit_prob[i]);
      }
      JANUS_DUMP("State", "Bit probabilities (percent)", "%s", string_bit_prob);

      JANUS_UTILS_MEMORY_ALLOCA_FREE(string_bit_prob);
    }
  }
}
