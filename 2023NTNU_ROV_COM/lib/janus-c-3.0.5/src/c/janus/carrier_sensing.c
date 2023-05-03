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
// Author: Luigi Elia D'Amaro, Giovanni Zappa                             *
//*************************************************************************

// ISO C headers.
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// JANUS headers.
#include <janus/carrier_sensing.h>
#include <janus/defaults.h>
#include <janus/utils/memory.h>

struct janus_carrier_sensing
{
  janus_rx_t rx;

  unsigned freq_vector_size;
  janus_real_t* freq_vector;

  janus_real_t* spectrogram;

  janus_real_t alpha_bn_fast;
  janus_real_t beta_bn_fast;
  janus_real_t alpha_bn_slow;
  janus_real_t beta_bn_slow;
  janus_real_t alpha_decay;
  janus_real_t beta_decay;

  janus_real_t actual_time;
  
  janus_int8_t triggering_flag;

  janus_real_t window[16 * JANUS_PREAMBLE_CHIP_OVERSAMPLING];
  unsigned window_capacity;
  unsigned window_size;
  unsigned window_index;
  janus_real_t window_power;
  janus_real_t window_power_decay;
  janus_real_t background_power;
};

janus_carrier_sensing_t
janus_carrier_sensing_new(janus_rx_t rx)
{
  janus_carrier_sensing_t carrier_sensing = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_carrier_sensing, 1);

  carrier_sensing->rx = rx;
  carrier_sensing->alpha_bn_fast = JANUS_REAL_CONST(3.0) / (JANUS_REAL_CONST(352.0) * JANUS_PREAMBLE_CHIP_OVERSAMPLING);
  carrier_sensing->beta_bn_fast  = JANUS_REAL_CONST(1.0) - carrier_sensing->alpha_bn_fast;
  carrier_sensing->alpha_bn_slow = JANUS_REAL_CONST(3.0) / (JANUS_REAL_CONST(352.0) * JANUS_PREAMBLE_CHIP_OVERSAMPLING * JANUS_REAL_CONST(3.0));
  carrier_sensing->beta_bn_slow  = JANUS_REAL_CONST(1.0) - carrier_sensing->alpha_bn_slow;
  carrier_sensing->alpha_decay = JANUS_REAL_CONST(3.0) / (JANUS_REAL_CONST(352.0) * JANUS_PREAMBLE_CHIP_OVERSAMPLING);
  carrier_sensing->beta_decay = JANUS_REAL_CONST(1.0) - carrier_sensing->alpha_decay;

  carrier_sensing->triggering_flag = 0;

  carrier_sensing->window_capacity = 16 * JANUS_PREAMBLE_CHIP_OVERSAMPLING;
  memset(&(carrier_sensing->window), 0, carrier_sensing->window_capacity);

  janus_rx_get_frequencies(rx, &(carrier_sensing->freq_vector_size), &(carrier_sensing->freq_vector));

  return carrier_sensing;
}

void
janus_carrier_sensing_free(janus_carrier_sensing_t carrier_sensing)
{
  JANUS_UTILS_MEMORY_FREE(carrier_sensing->freq_vector);
  JANUS_UTILS_MEMORY_FREE(carrier_sensing->spectrogram);
  JANUS_UTILS_MEMORY_FREE(carrier_sensing);
}

int
janus_carrier_sensing_execute(janus_carrier_sensing_t carrier_sensing, janus_real_t* time)
{
  int rv = 0;
  unsigned data_size;
  janus_real_t last_time;
  janus_real_t step_time;

  *time = JANUS_REAL_CONST(0.0);

  if (!janus_rx_has_detected(carrier_sensing->rx) &&
      janus_rx_get_channel_spectrogram(carrier_sensing->rx, &(carrier_sensing->spectrogram), &data_size, &last_time, &step_time))
  {
    unsigned i, k;

#if 0
    for (i = 0; i < data_size; ++i)
    {
      fprintf(stderr, "%8.6f ", last_time - step_time * (data_size - 1 - i));
      for (k = 0; k != carrier_sensing->freq_vector_size; ++k)
      {
        fprintf(stderr, "% 6.3f ", *(carrier_sensing->spectrogram + (k * data_size + i)));
      }
      fprintf(stderr, "\n");
    }
#endif

    for (i = 0; i < data_size; ++i)
    {
      carrier_sensing->actual_time = last_time - (step_time * (data_size - 1 - i));

      janus_real_t power = JANUS_REAL_CONST(0.0);
      for (k = 0; k != carrier_sensing->freq_vector_size; ++k)
      {
        janus_real_t s = *(carrier_sensing->spectrogram + (k * data_size + i));
        power += s * s;
      }

      // Compute background power.
      if (! carrier_sensing->triggering_flag) {
        carrier_sensing->background_power = carrier_sensing->alpha_bn_fast * power + carrier_sensing->beta_bn_fast * carrier_sensing->background_power;
      } else {
        carrier_sensing->background_power = carrier_sensing->alpha_bn_slow * power + carrier_sensing->beta_bn_slow * carrier_sensing->background_power;
      }

      // Compute window power.
      carrier_sensing->window_power += (power - carrier_sensing->window[carrier_sensing->window_index]) / carrier_sensing->window_capacity;

      if (carrier_sensing->window_power > carrier_sensing->window_power_decay) {
        carrier_sensing->window_power_decay = carrier_sensing->window_power;
      } else {
        carrier_sensing->window_power_decay = carrier_sensing->beta_decay * carrier_sensing->window_power_decay;
      }
            
      if ((carrier_sensing->window_size >= carrier_sensing->window_capacity - 1) &&
          (carrier_sensing->window_power_decay > carrier_sensing->background_power * JANUS_REAL_CONST(2.0)))
      {
        if (*time == JANUS_REAL_CONST(0.0))
          *time = last_time - (step_time * (data_size - 1 - i));
        carrier_sensing->triggering_flag = 1;
        rv = 1;
      } else {
        carrier_sensing->triggering_flag = 0;
      }

      // Update window values.
      carrier_sensing->window[carrier_sensing->window_index] = power;

      // Update window index.
      carrier_sensing->window_index++;
      if (carrier_sensing->window_index == carrier_sensing->window_capacity)
        carrier_sensing->window_index = 0;

      // Update window size.
      if (carrier_sensing->window_size < carrier_sensing->window_capacity)
        carrier_sensing->window_size++;

      if (carrier_sensing->window_size <= carrier_sensing->window_capacity - 1 ||
          carrier_sensing->background_power <= JANUS_REAL_CONST(0.00000000000001))  // equivalent resolution of 24 bit ADC
      {
        carrier_sensing->background_power = carrier_sensing->window_power;
        //fprintf(stderr, "background_power %f\n", carrier_sensing->background_power);
      }
    }
    
    if (carrier_sensing->window_size < carrier_sensing->window_capacity)
      rv = -1;
  }

  return rv;
}

janus_real_t
janus_carrier_sensing_background_power(janus_carrier_sensing_t carrier_sensing)
{
  return carrier_sensing->background_power;
}

janus_real_t
janus_carrier_sensing_window_power(janus_carrier_sensing_t carrier_sensing)
{
  return carrier_sensing->window_power_decay;
}

janus_real_t
janus_carrier_sensing_time(janus_carrier_sensing_t carrier_sensing)
{
  return carrier_sensing->actual_time;
}

void
janus_carrier_sensing_reset(janus_carrier_sensing_t carrier_sensing)
{
  carrier_sensing->window_size = 0;
  carrier_sensing->window_index = 0;
}
