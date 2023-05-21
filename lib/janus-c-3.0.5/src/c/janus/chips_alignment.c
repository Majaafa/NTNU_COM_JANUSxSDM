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
// Authors: Luigi Elia D'Amaro, Giovanni Zappa                            *
//*************************************************************************

// ISO C headers.
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// JANUS headers.
#include <janus/utils/imath.h>
#include <janus/chips_alignment.h>
#include <janus/defaults.h>
#include <janus/goertzel.h>
#include <janus/utils/hamming_window.h>
#include <janus/utils/memory.h>

//! Baseband fifo size (in samples).
#define BBAND_FIFO_SIZE 16 * 1024

struct janus_chips_alignment
{
  //! Sampling frequency.
  unsigned fs;
  //! Number of frequencies.
  unsigned freq_vector_size;
  //! Frequencies.
  janus_real_t* freq_vector;
  //! Preamble size (number of chips in preamble).
  unsigned n_chips;
  //! Preamble pattern (order of frequencies within preamble).
  unsigned* c_order;
  //! Doppler max speed (in meters/sec).
  janus_real_t max_speed_ms;

  //! Delay from baseband signal and chips alignment [s].
  janus_real_t align_delay;
  
  //! Chip oversampling.
  unsigned c_ovs;
  //! Chip oversampling offset.
  janus_real_t c_ovs_offset;

  //! Real samples counter.
  janus_hiprecision_t counter;

  janus_complex_t* x_win;
  janus_real_t* abs_dft;

  //! Goertzel filter window length.
  unsigned gf_size;
  //! Goertzel algorithm.
  janus_goertzel_t goertzel;
  
  //! Hamming window.
  janus_real_t* ham_win;

  //! Input buffer.
  janus_complex_t* ibuffer;
  //! Input buffer read index.
  janus_uint64_t ibuffer_read_index;
  //! Input buffer write index.
  janus_uint64_t ibuffer_write_index;
  //! Input buffer size.
  unsigned ibuffer_size;
  //! Input buffer size in bytes.
  unsigned ibuffer_size_bytes;
  //! Input buffer copied data size in bytes.
  unsigned ibuffer_copied_bytes;

  //! Internal matrix containing the spectrograms of last n_chips chips (freq_vector_size rows, n_chips * c_ovs columns).
  janus_real_t** matrix;
  //! Internal matrix columns number.
  unsigned matrix_n_columns;
  //! Internal matrix current column read index.
  unsigned matrix_read_index;
  //! Internal matrix current column write index.
  unsigned matrix_write_index;
  //! Internal matrix spectrogram column read index.
  unsigned matrix_spectrogram_index;
};

static unsigned
next_power_of_2(unsigned v)
{
  unsigned r = 1;
  
  while (r < v)
    r <<= 1;
  
  return r;
}

janus_chips_alignment_t
janus_chips_alignment_new(unsigned fs, janus_real_t chip_duration, unsigned freq_vector_size, janus_real_t freq_vector[], unsigned n_chips, unsigned c_order[], janus_real_t max_speed_ms)
{
  janus_chips_alignment_t chips_alignment = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_chips_alignment, 1);
  janus_real_t max_freq;
  unsigned chip_size;
  unsigned i, k;

  chips_alignment->fs = fs;
  chips_alignment->freq_vector_size = freq_vector_size;
  chips_alignment->freq_vector = JANUS_UTILS_MEMORY_NEW_ZERO(janus_real_t, freq_vector_size);
  memcpy(chips_alignment->freq_vector, freq_vector, sizeof(janus_real_t) * freq_vector_size);
  chips_alignment->n_chips = n_chips;
  chips_alignment->c_order = JANUS_UTILS_MEMORY_NEW_ZERO(unsigned, n_chips);
  memcpy(chips_alignment->c_order, c_order, sizeof(unsigned) * n_chips);
  chips_alignment->max_speed_ms = max_speed_ms;

  chips_alignment->c_ovs = JANUS_PREAMBLE_CHIP_OVERSAMPLING;
  chips_alignment->c_ovs_offset = (chip_duration * fs) / chips_alignment->c_ovs;
  chips_alignment->counter = JANUS_HIPRECISION_CONST(0.0);

  max_freq = freq_vector[0];
  for (i = 1; i < freq_vector_size; i++)
  {
    if (max_freq < freq_vector[i])
    {
      max_freq = freq_vector[i];
    }
  }
  chip_size = (unsigned)(chip_duration * fs);
  chips_alignment->gf_size = (unsigned)JANUS_FLOOR(fs * (1540 * chip_duration) / ((chip_duration * max_freq + 1) * max_speed_ms + 1540));
  chips_alignment->gf_size = (unsigned)JANUS_FMAX(chips_alignment->gf_size, 0.75 * chip_size);

  chips_alignment->align_delay = chips_alignment->gf_size * (JANUS_REAL_CONST(1.0) - JANUS_REAL_CONST(1.0) / JANUS_PREAMBLE_CHIP_OVERSAMPLING) / (JANUS_REAL_CONST(2.0) * chips_alignment->fs);

  chips_alignment->x_win = JANUS_UTILS_MEMORY_NEW_ZERO(janus_complex_t, chips_alignment->gf_size);
  chips_alignment->abs_dft = JANUS_UTILS_MEMORY_NEW_ZERO(janus_real_t, chips_alignment->freq_vector_size);

  chips_alignment->goertzel = janus_goertzel_new(fs, freq_vector_size, freq_vector, chips_alignment->gf_size);

  chips_alignment->ham_win = JANUS_UTILS_MEMORY_NEW_ZERO(janus_real_t, chips_alignment->gf_size);
  janus_utils_hamming_window(chips_alignment->gf_size, chips_alignment->ham_win);

  chips_alignment->ibuffer_size = (unsigned)JANUS_FMAX(chips_alignment->c_ovs_offset * (chips_alignment->c_ovs - 1) + chips_alignment->gf_size, 2 * BBAND_FIFO_SIZE);
  chips_alignment->ibuffer_size_bytes = chips_alignment->ibuffer_size * sizeof(janus_complex_t);
  chips_alignment->ibuffer = JANUS_UTILS_MEMORY_NEW_ZERO(janus_complex_t, chips_alignment->ibuffer_size);
  chips_alignment->ibuffer_read_index = 0;
  chips_alignment->ibuffer_write_index = 0;
  chips_alignment->ibuffer_copied_bytes = 0;

  chips_alignment->matrix_n_columns = next_power_of_2((chips_alignment->n_chips + 1) * chips_alignment->c_ovs);
  chips_alignment->matrix = JANUS_UTILS_MEMORY_NEW_ZERO(janus_real_t*, chips_alignment->freq_vector_size);
  for (k = 0; k < chips_alignment->freq_vector_size; k++)
  {
    chips_alignment->matrix[k] = JANUS_UTILS_MEMORY_NEW_ZERO(janus_real_t, chips_alignment->matrix_n_columns);
  }
  chips_alignment->matrix_read_index = 0;
  chips_alignment->matrix_write_index = 0;
  chips_alignment->matrix_spectrogram_index = 0;

  return chips_alignment;
}

void
janus_chips_alignment_free(janus_chips_alignment_t chips_alignment)
{
  unsigned i;
  for (i = 0; i < chips_alignment->freq_vector_size; i++)
  {
    JANUS_UTILS_MEMORY_FREE(chips_alignment->matrix[i]);
  }
  JANUS_UTILS_MEMORY_FREE(chips_alignment->matrix);
  JANUS_UTILS_MEMORY_FREE(chips_alignment->ibuffer);
  janus_goertzel_free(chips_alignment->goertzel);
  JANUS_UTILS_MEMORY_FREE(chips_alignment->x_win);
  JANUS_UTILS_MEMORY_FREE(chips_alignment->abs_dft);
  JANUS_UTILS_MEMORY_FREE(chips_alignment->ham_win);
  JANUS_UTILS_MEMORY_FREE(chips_alignment->c_order);
  JANUS_UTILS_MEMORY_FREE(chips_alignment->freq_vector);
  JANUS_UTILS_MEMORY_FREE(chips_alignment);
}

void
janus_chips_alignment_reset(janus_chips_alignment_t chips_alignment)
{
  chips_alignment->counter = JANUS_HIPRECISION_CONST(0.0);

  chips_alignment->ibuffer_read_index = 0;
  chips_alignment->ibuffer_write_index = 0;
  chips_alignment->ibuffer_copied_bytes = 0;

  chips_alignment->matrix_read_index = 0;
  chips_alignment->matrix_write_index = 0;
  chips_alignment->matrix_spectrogram_index = 0;
}

unsigned
janus_chips_alignment_execute(janus_chips_alignment_t chips_alignment, janus_utils_fifo_t ififo, unsigned ififo_offset, janus_utils_fifo_t ofifo)
{
  unsigned offset_bytes = ififo_offset * sizeof(janus_complex_t);
  unsigned input_bytes = ((janus_utils_fifo_get_size(ififo) - offset_bytes) / sizeof(janus_complex_t)) * sizeof(janus_complex_t);
  unsigned ibuffer_write_to_end_bytes;
  unsigned copied_bytes_to_end;
  void* ibuffer_write_pointer;
  unsigned copied_bytes_from_begin;

  while (input_bytes > 0)
  {
    unsigned ibuffer_full_bytes = (unsigned)(chips_alignment->ibuffer_write_index - chips_alignment->ibuffer_read_index) * sizeof(janus_complex_t);
    unsigned ibuffer_empty_bytes = chips_alignment->ibuffer_size_bytes - ibuffer_full_bytes;

    unsigned copied_bytes = janus_umin(input_bytes, ibuffer_empty_bytes);

    input_bytes -= copied_bytes;

    // Read from ififo and write to ibuffer from chips_alignment->ibuffer_write_index to the end.
    ibuffer_write_to_end_bytes = (chips_alignment->ibuffer_size - (chips_alignment->ibuffer_write_index & (chips_alignment->ibuffer_size - 1))) * sizeof(janus_complex_t);
    copied_bytes_to_end = janus_umin(copied_bytes, ibuffer_write_to_end_bytes);
    ibuffer_write_pointer = chips_alignment->ibuffer + (chips_alignment->ibuffer_write_index & (chips_alignment->ibuffer_size - 1));
    janus_utils_fifo_peek_offset(ififo, ibuffer_write_pointer, copied_bytes_to_end, offset_bytes);

    // Read from ififo and write to ibuffer from the beggining to copied_bytes - copied_bytes_to_end.
    copied_bytes_from_begin = copied_bytes - copied_bytes_to_end;
    janus_utils_fifo_peek_offset(ififo, chips_alignment->ibuffer, copied_bytes_from_begin, offset_bytes + copied_bytes_to_end);
    
    offset_bytes += copied_bytes;

#if 0
    unsigned s = (chips_alignment->ibuffer_write_index & (chips_alignment->ibuffer_size - 1));
    for (unsigned i = 0; i < copied_bytes_to_end / sizeof(janus_complex_t); i++)
      printf("%+.15f %+.15f\n", chips_alignment->ibuffer[s + i][0], chips_alignment->ibuffer[s + i][1]);
    for (unsigned i = 0; i < copied_bytes_from_begin / sizeof(janus_complex_t); i++)
      printf("%+.15f %+.15f\n", chips_alignment->ibuffer[i][0], chips_alignment->ibuffer[i][1]);
#endif

    // Move ibuffer write index.
    chips_alignment->ibuffer_write_index += copied_bytes / sizeof(janus_complex_t);

    while (chips_alignment->ibuffer_write_index - chips_alignment->ibuffer_read_index > chips_alignment->gf_size)
    {
      unsigned i, k, n, t;
      janus_real_t curr_value, prev_value;
      unsigned long matrix_unread_size;

      // Get the right window of input values and multiply it for the hamming window.
      i = (chips_alignment->ibuffer_read_index & (chips_alignment->ibuffer_size - 1));
      for (n = 0; n < chips_alignment->gf_size; n++)
      {
        janus_complex_mul_real(*(chips_alignment->ibuffer + i), chips_alignment->ham_win[n], chips_alignment->x_win[n]);
        if (++i == chips_alignment->ibuffer_size)
        {
          i = 0;
        }
      }

      // Execute the goertzel algorithm, getting the modulus of the output values.
      janus_goertzel_abs_execute(chips_alignment->goertzel, chips_alignment->x_win, chips_alignment->abs_dft);

      i = chips_alignment->matrix_write_index & (chips_alignment->matrix_n_columns - 1);
      t = (chips_alignment->matrix_write_index - 1) & (chips_alignment->matrix_n_columns - 1);
      for (k = 0; k < chips_alignment->freq_vector_size; k++)
      {
        // Store the goertzel algorithm outputs in the matrix.
        curr_value = chips_alignment->abs_dft[k] / (janus_real_t)chips_alignment->n_chips;
        chips_alignment->matrix[k][i] = curr_value;

        // Max filter applied to previous value that becomes equal to the maximum between previous and current value.
        prev_value = chips_alignment->matrix[k][t];
        chips_alignment->matrix[k][t] = JANUS_FMAX(prev_value, curr_value);
      }

#if 0
      {
        static int first = 1;
        if (!first)
        {
          for (k = 0; k < chips_alignment->freq_vector_size; k++)
          {
            printf("% 6.3f ", chips_alignment->matrix[k][t]);
          }
          printf("\n");
        }
        else
          first = 0;
      }
#endif

      chips_alignment->matrix_write_index++;

      // Update counter and ibuffer read index.
      chips_alignment->counter += chips_alignment->c_ovs_offset;
      chips_alignment->ibuffer_read_index = (janus_uint64_t)chips_alignment->counter;

      matrix_unread_size = chips_alignment->matrix_write_index - chips_alignment->matrix_read_index;

      while (matrix_unread_size >= (chips_alignment->n_chips + 1) * chips_alignment->c_ovs)
      {
        janus_real_t output = 0.0;
        unsigned c;
        t = chips_alignment->matrix_read_index & (chips_alignment->matrix_n_columns - 1);
        for (c = 0; c < chips_alignment->n_chips; c++)
        {
          k = chips_alignment->c_order[c];
          output += chips_alignment->matrix[k][t];
          t = (t + chips_alignment->c_ovs) & (chips_alignment->matrix_n_columns - 1);
        }
        chips_alignment->matrix_read_index++;
        matrix_unread_size = chips_alignment->matrix_write_index - chips_alignment->matrix_read_index;

#if 0
        printf("CA:% 12.8f\n", output);
#endif

        if (janus_utils_fifo_put(ofifo, &output, sizeof(janus_real_t)) != sizeof(janus_real_t))
        {
          fprintf(stderr, "WARNING: chips alignment overrun.\n");
        }
      }
    }
  }

  return offset_bytes / sizeof(janus_complex_t) - ififo_offset;
}

void
janus_chips_alignment_get_frequencies(janus_chips_alignment_t chips_alignment, unsigned* freq_vector_size, janus_real_t** freq_vector)
{
  *freq_vector_size = chips_alignment->freq_vector_size;
  
  *freq_vector = JANUS_UTILS_MEMORY_REALLOC(*freq_vector, janus_real_t, *freq_vector_size);

  memcpy(*freq_vector, chips_alignment->freq_vector, sizeof(janus_real_t) * (*freq_vector_size));
}

unsigned
janus_chips_alignment_get_preamble_size(janus_chips_alignment_t chips_alignment)
{
  return chips_alignment->n_chips;
}

janus_hiprecision_t
janus_chips_alignment_get_bb_counter(janus_chips_alignment_t chips_alignment)
{
  return chips_alignment->counter;
}

unsigned
janus_chips_alignment_get_goertzel_windows_size(janus_chips_alignment_t chips_alignment)
{
  return chips_alignment->gf_size;
}

janus_real_t
janus_chips_alignment_get_align_delay(janus_chips_alignment_t chips_alignment)
{
  return chips_alignment->align_delay;
}

unsigned
janus_chips_alignment_get_spectrogram(janus_chips_alignment_t chips_alignment, janus_real_t** spectrogram)
{
  int data_size = chips_alignment->matrix_write_index - chips_alignment->matrix_spectrogram_index - JANUS_PREAMBLE_CHIP_OVERSAMPLING;
  unsigned in_capacity = chips_alignment->matrix_n_columns;
  unsigned i;

  if (data_size <= 0)
    return 0;

  if (data_size > 8 * JANUS_PREAMBLE_CHIP_OVERSAMPLING)
  {
    chips_alignment->matrix_spectrogram_index += data_size - 8 * JANUS_PREAMBLE_CHIP_OVERSAMPLING;
    data_size = 8 * JANUS_PREAMBLE_CHIP_OVERSAMPLING;
  }
  
  *spectrogram = JANUS_UTILS_MEMORY_REALLOC(*spectrogram, janus_real_t, chips_alignment->freq_vector_size * data_size);

  for (i = 0; i != chips_alignment->freq_vector_size; ++i)
  {
    janus_real_t* out_data = (*spectrogram) + (i * data_size);
    janus_real_t* in_data = chips_alignment->matrix[i];
    unsigned in_offset = chips_alignment->matrix_spectrogram_index;
    
    // Get the data from "in_data", starting from "in_offset" until the end.
    unsigned fsize = JANUS_MIN(data_size, (int)(in_capacity - (in_offset & (in_capacity - 1))));
    memcpy(out_data, in_data + (in_offset & (in_capacity - 1)), fsize * sizeof(janus_real_t));

    // Get the rest (if any) from the beginning of "in_data".
    memcpy(out_data + fsize, in_data, (data_size - fsize) * sizeof(janus_real_t));
  }

#if 0
  unsigned k = 0;
  for (k = 0; k < data_size; ++k)
  {
    for (i = 0; i != chips_alignment->freq_vector_size; ++i)
    {
      fprintf(stderr, "% 6.3f ", *((*spectrogram) + (i * data_size + k)));
    }
    fprintf(stderr, "\n");
  }
#endif

  chips_alignment->matrix_spectrogram_index += data_size;

  return data_size;
}
