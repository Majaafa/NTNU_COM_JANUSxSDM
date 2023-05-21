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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// JANUS headers.
#include <janus/utils/go_cfar.h>
#include <janus/utils/memory.h>
#include <janus/defaults.h>

#define SUM_RESET 300000

typedef enum
{
  //! Searching for the first input value exceeding the threshold.
  STATE_FIRST_DETECTION,
  //! Searching for the maximum input value exceeding the threshold, within channel spread past the first one.
  STATE_MAXIMUM_DETECTION,
  //! Maximum detected.
  STATE_MAXIMUM_DETECTED,
} janus_go_cfar_state_t;

struct janus_go_cfar
{
  //! Window size.
  unsigned n;
  //! Half the window size.
  unsigned hn;
  //! Guard size.
  unsigned g;
  //! Half the guard size.
  unsigned hg;
  //! Threshold.
  janus_real_t t;
  //! Window correction.
  janus_real_t w;
  //! Stream size.
  unsigned s;
  //! Channel spread.
  unsigned c;
  //! Chip oversampling.
  unsigned o;
  //! Skip offset.
  unsigned k;

  //! Absolute counter.
  janus_uint64_t counter;
  
  //! Internal buffer.
  janus_real_t* buffer;
  //! Internal buffer's size in bytes.
  unsigned buffer_size_bytes;
  //! First detection buffer size in bytes.
  unsigned fd_buffer_size_bytes;
  //! Maximum detection buffer size in bytes.
  unsigned md_buffer_size_bytes;
  //! Windows size in bytes.
  unsigned window_size_bytes;

  //! Sum FIFO size in bytes.
  unsigned sum_fifo_size_bytes;
  //! Sum FIFO.
  janus_utils_fifo_t sum_fifo;
  //! Sum iterations from the last refresh of sum FIFO.
  unsigned sum_iterations;
  //! Sum refresh.
  janus_real_t sum_reset;

  //! First detection
  janus_uint64_t first_detection;
  //! Remaning detection offset.
  janus_real_t detection_offset;

  //! Peak detected energy
  janus_real_t detection_peak_energy;
  //! Background GO energy at first detection
  janus_real_t detection_background_energy;
  
  //! Current GO CFAR state.
  janus_go_cfar_state_t state;
};

static void
janus_go_cfar_first_detection(janus_go_cfar_t go_cfar, janus_utils_fifo_t fifo)
{
  unsigned fifo_size_bytes = janus_utils_fifo_get_size(fifo);
  
  while (fifo_size_bytes >= ((go_cfar->hn + 1 + go_cfar->k) * sizeof(janus_real_t)))
  {
    unsigned peek_size_bytes = 0;
    unsigned peek_offset_bytes = 0;
    unsigned mov_avg_size = go_cfar->hn - go_cfar->hg;
    
    /* Initialize after reset */
    if (go_cfar->counter == 0)
    {
      int i = 0;
      janus_real_t z_r = 0;

      peek_size_bytes = (go_cfar->hn  * sizeof(janus_real_t));
      janus_utils_fifo_peek_offset(fifo, go_cfar->buffer, peek_size_bytes, (go_cfar->k * sizeof(janus_real_t)));
      peek_offset_bytes = peek_size_bytes;

      // compute first moving average on right window
      for (i = 0; i < mov_avg_size; i++){
        janus_real_t add = go_cfar->buffer[i];
        z_r += add;
      }
      janus_utils_fifo_put(go_cfar->sum_fifo, &z_r, sizeof(janus_real_t));

      // proceed for the firs half window
      for (i = mov_avg_size; i < go_cfar->hn; i++){
        janus_real_t add = go_cfar->buffer[i];
        janus_real_t sub = go_cfar->buffer[i - mov_avg_size];
        z_r += (add - sub);
        janus_utils_fifo_put(go_cfar->sum_fifo, &z_r, sizeof(janus_real_t));
      }
      go_cfar->sum_iterations += go_cfar->hn - mov_avg_size;
    }

    // avoiding overlap between left and right sums 
    peek_size_bytes = JANUS_MIN((go_cfar->hn + go_cfar->hg + 1) * sizeof(janus_real_t), fifo_size_bytes);

    // adding peek_offset_bytes to skip data might be already in buffer
    janus_utils_fifo_peek_offset(fifo, (char*) (go_cfar->buffer) + peek_offset_bytes,
                                 peek_size_bytes - peek_offset_bytes, peek_offset_bytes + (go_cfar->k * sizeof(janus_real_t)));
    
    {
      unsigned iterations = (peek_size_bytes / sizeof(janus_real_t)) - go_cfar->hn;

      unsigned     s = 0;
      janus_real_t z_go = 0;
      janus_real_t z = 0;
      janus_real_t add = 0;
      janus_real_t sub = 0;

      janus_real_t z_r = 0;
      janus_real_t z_l = 0;
      janus_real_t* z_lp, *z_lv = JANUS_UTILS_MEMORY_ALLOCA(janus_real_t, iterations);
      z_lp = z_lv;

      // get z_lv if already computed from sum_fifo
      if ((go_cfar->counter + iterations) > (go_cfar->hn + go_cfar->hg + 1))
      {
        unsigned valid_z_lv = JANUS_MIN(iterations, go_cfar->counter + iterations - (go_cfar->hn + go_cfar->hg + 1));
        janus_utils_fifo_peek(go_cfar->sum_fifo, z_lv, valid_z_lv * sizeof(janus_real_t));
      }

      // get last computed moving average or recompute if exceeded the SUM_RESET
      if (go_cfar->sum_iterations > SUM_RESET)
      {
        unsigned i;
        for (i = go_cfar->hg; i < go_cfar->hn; i++)
        {
          add = go_cfar->buffer[i];
          z_r += add;
        }
        go_cfar->sum_iterations = 0;
      }
      else
      {
        janus_utils_fifo_peek_offset(go_cfar->sum_fifo, &z_r, sizeof(janus_real_t), janus_utils_fifo_get_size(go_cfar->sum_fifo) - sizeof(janus_real_t));
      }

      for (s = 0; s < iterations; s++)
      {
        // compute the next z_r
        {
          unsigned i = go_cfar->hn + s;
        
          add = go_cfar->buffer[i];
          sub = go_cfar->buffer[i - mov_avg_size];
          z_r += (add - sub);
          go_cfar->sum_iterations++;
          janus_utils_fifo_put(go_cfar->sum_fifo, &z_r, sizeof(janus_real_t));
        }

        // if available get z_l from the array z_lv
        if ((go_cfar->counter + s) > (go_cfar->hn + go_cfar->hg + 1))
        {
          z_l = *z_lp++;
        }
        
        z = go_cfar->buffer[s];

        z_go = JANUS_FMAX(z_l, z_r - z * go_cfar->w) * go_cfar->t / mov_avg_size;

#if 1
        if (z > z_go && z > JANUS_REAL_EPSILON)
        {
          go_cfar->first_detection = go_cfar->counter;
          go_cfar->state = STATE_MAXIMUM_DETECTION;

          if (z_lp > z_lv)
          {
            // cosider background energy the left cell
            // TODO: use z_l value before the cell is contaminated by the preamble
            go_cfar->detection_background_energy = z_l / mov_avg_size;
            janus_utils_fifo_skip(go_cfar->sum_fifo, (z_lp - z_lv) * sizeof(janus_real_t));
          }
          janus_utils_fifo_skip(fifo, JANUS_MAX(0, s * sizeof(janus_real_t)));
  
          return;
        }
#else
        // printf("%0.15f\n", JANUS_FMAX(z_l, z_r - z * go_cfar->w) / mov_avg_size);
        // Printing left average, right average and test value
        printf("%09.3f\t%-16.15f\t%-16.15f\t%-16.15f\t\n", (go_cfar->counter * 6.25e-3 / 4), z_l, z_r,  z);
#endif

        go_cfar->counter++;
      }
      
      // skip consumed data from fifos
      // TODO: use a longer fifo to extract background noise
      if (go_cfar->counter <= go_cfar->o) {
        go_cfar->k = go_cfar->counter;
      }
      else {
        if (go_cfar->k < go_cfar->o) {
          janus_utils_fifo_skip(fifo, (iterations - ((int)go_cfar->o - (int)go_cfar->k)) * sizeof(janus_real_t));
          go_cfar->k = go_cfar->o;
        }
        else {
          janus_utils_fifo_skip(fifo, iterations * sizeof(janus_real_t));
        }
      }

      if (z_lp > z_lv)
      {
        janus_utils_fifo_skip(go_cfar->sum_fifo, (z_lp - z_lv) * sizeof(janus_real_t));
      }

      fifo_size_bytes -= peek_size_bytes;
      JANUS_UTILS_MEMORY_ALLOCA_FREE(z_lv);
    }
  }
}

static unsigned
janus_go_cfar_maximum_detection(janus_go_cfar_t go_cfar, janus_utils_fifo_t fifo)
{
  janus_real_t max = -1;
  unsigned idx = 0, c;

  if (janus_utils_fifo_get_size(fifo) >= go_cfar->md_buffer_size_bytes)
  {
    janus_utils_fifo_peek(fifo, go_cfar->buffer, go_cfar->md_buffer_size_bytes);

    for (c = 0; c < go_cfar->c; c++)
    {
      if (go_cfar->buffer[c] > max)
      {
        max = go_cfar->buffer[c];
        idx = c;
      }
    }
    go_cfar->detection_peak_energy = max;

#if 1
    printf("peak: %.15f\n", go_cfar->detection_peak_energy);
#endif
  
    if (max > 0)
    {
      const int start = idx - go_cfar->o;
      unsigned i_start = JANUS_MAX(0, start);
      unsigned i_end = idx;
      janus_real_t step_max = go_cfar->buffer[idx];
      janus_real_t step_sum = step_max;
      unsigned i;
      for (i = i_start; i != i_end; ++i)
      {
        step_sum += go_cfar->buffer[i];
      }

      go_cfar->detection_offset = JANUS_REAL_CONST(1.0) - step_sum / step_max;
      go_cfar->counter += idx;

      janus_utils_fifo_skip(fifo, sizeof(janus_real_t) * idx);
      go_cfar->state = STATE_MAXIMUM_DETECTED;
    }
  }

  return idx;
}

janus_go_cfar_t
janus_go_cfar_new(unsigned n, unsigned g, janus_real_t w, unsigned s, unsigned c, unsigned o)
{
  janus_go_cfar_t go_cfar = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_go_cfar, 1);

  if ((n % 2) != 0)
    ++n;

  if ((g % 2) != 0)
    ++g;

  go_cfar->n = n;
  go_cfar->hn = n / 2;
  go_cfar->g = g;
  go_cfar->hg = g / 2;
  go_cfar->t = 0;
  go_cfar->w = w * (go_cfar->hn - go_cfar->hg);
  go_cfar->s = s;
  go_cfar->c = c;
  go_cfar->o = o;
  
  go_cfar->window_size_bytes = n * sizeof(janus_real_t);
  go_cfar->fd_buffer_size_bytes = go_cfar->window_size_bytes + (s - 1) * sizeof(janus_real_t);
  go_cfar->md_buffer_size_bytes = go_cfar->window_size_bytes + (c - 1) * sizeof(janus_real_t);
  go_cfar->buffer_size_bytes = JANUS_MAX(go_cfar->fd_buffer_size_bytes, go_cfar->md_buffer_size_bytes);
  go_cfar->buffer = (janus_real_t*)malloc(go_cfar->buffer_size_bytes);

  go_cfar->sum_fifo_size_bytes = (go_cfar->n + go_cfar->hg + 1) * sizeof(janus_real_t);
  go_cfar->sum_fifo = janus_utils_fifo_new(go_cfar->sum_fifo_size_bytes);

  janus_go_cfar_reset(go_cfar);

  return go_cfar;
}

void
janus_go_cfar_free(janus_go_cfar_t go_cfar)
{
  janus_utils_fifo_free(go_cfar->sum_fifo);
  free(go_cfar->buffer);
  JANUS_UTILS_MEMORY_FREE(go_cfar);
}

void
janus_go_cfar_reset(janus_go_cfar_t go_cfar)
{
  go_cfar->k = 0;

  go_cfar->first_detection = 0;
  go_cfar->detection_offset = 0;
  go_cfar->counter = 0;

  // Detection background energy negative when unavailable
  go_cfar->detection_background_energy = -1;
  // Detection peak energy negative when unavailable
  go_cfar->detection_peak_energy = -1;

  go_cfar->state = STATE_FIRST_DETECTION;
  //memset(go_cfar->buffer, 0, go_cfar->buffer_size_bytes);

  go_cfar->sum_iterations = 0;
  janus_utils_fifo_reset(go_cfar->sum_fifo);
}

unsigned
janus_go_cfar_execute(janus_go_cfar_t go_cfar, janus_utils_fifo_t fifo)
{
  unsigned idx = 0;

  if (go_cfar->state == STATE_FIRST_DETECTION)
  {
    janus_go_cfar_first_detection(go_cfar, fifo);
  }
  else if (go_cfar->state == STATE_MAXIMUM_DETECTION)
  {
    idx = janus_go_cfar_maximum_detection(go_cfar, fifo);
  }
  
  return idx;
}

int
janus_go_cfar_has_detected(janus_go_cfar_t go_cfar)
{
  return (go_cfar->state == STATE_MAXIMUM_DETECTION || go_cfar->state == STATE_MAXIMUM_DETECTED) ? 1 : 0;
}

int
janus_go_cfar_detection_competed(janus_go_cfar_t go_cfar)
{
  return (go_cfar->state == STATE_MAXIMUM_DETECTED) ? 1 : 0;
}

unsigned
janus_go_cfar_get_window_size(janus_go_cfar_t go_cfar)
{
  return go_cfar->n;
}

janus_real_t
janus_go_cfar_get_detection_offset(janus_go_cfar_t go_cfar)
{
  return go_cfar->detection_offset - go_cfar->k;
}

janus_uint64_t
janus_go_cfar_get_counter(janus_go_cfar_t go_cfar)
{
  return go_cfar->counter;
}

janus_uint64_t 
janus_go_cfar_get_first_detection(janus_go_cfar_t go_cfar)
{
  return go_cfar->first_detection;
}

janus_real_t 
janus_go_cfar_get_detection_peak_energy(janus_go_cfar_t go_cfar)
{
  return go_cfar->detection_peak_energy;
}

janus_real_t 
janus_go_cfar_get_detection_background_energy(janus_go_cfar_t go_cfar)
{
  return go_cfar->detection_background_energy;
}

void
janus_go_cfar_set_threshold(janus_go_cfar_t go_cfar, janus_real_t detection_threshold)
{
  go_cfar->t = detection_threshold;
}
