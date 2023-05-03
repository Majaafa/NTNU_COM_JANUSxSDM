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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// FFTW3 headers.
#include <fftw3.h>

// JANUS headers.
#include <janus/defaults.h>
#include <janus/error.h>
#include <janus/utils/imath.h>
#include <janus/rx.h>
#include <janus/trellis.h>
#include <janus/hop_index.h>
#include <janus/chips_alignment.h>
#include <janus/doppler.h>
#include <janus/viterbi.h>
#include <janus/demodulator.h>
#include <janus/deinterleave.h>
#include <janus/codec/codecs.h>
#include <janus/types.h>
#include <janus/utils/memory.h>
#include <janus/utils/go_cfar.h>
#include <janus/utils/fifo.h>
#include <janus/utils/bin2dec.h>

//! Default Viterbi trace back length.
#define DEFAULT_VITERBI_TBK_LEN (9 * 5)
//! Baseband fifo size (in samples).
#define BBAND_FIFO_SIZE 16 * 1024
//! Blackout time.
#define JANUS_BLACKOUT_TIME (rx->pset->chip_dur)

typedef enum
{
  //! Searching for channel acquisition.
  STATE_IDLE,
  //! Searching for 32 chip preamble.
  STATE_32_CHIP_PREAMBLE,
  //! Estimating doppler.
  STATE_DOPPLER_ESTIMATION,
  //! Decoding packet.
  STATE_PACKET,
  //! Decoding optional packet cargo.
  STATE_PACKET_CARGO,
  //! Blackout after decoding.
  STATE_BLACKOUT
} janus_rx_execution_state_t;

struct janus_rx
{
  //! Verbose.
  janus_uint8_t verbose;
  //! Sampling frequency.
  unsigned bband_fs;
  //! Viterbi decoder.
  janus_viterbi_t viterbi;
  //! Viterbi decoder output buffer (bits).
  janus_uint8_t* viterbi_out_bits;
  //! Viterbi decoder output buffer (bytes).
  janus_uint8_t* viterbi_out_bytes;
  //! Baseband signal FIFO.
  janus_utils_fifo_t bband_fifo;
  //! Baseband samples counter from start.
  janus_uint64_t bband_time_counter;
  //! Baseband samples counter at cfar reset.
  janus_int64_t bband_cfar_init_counter;
  //! Blackout downcounter.
  unsigned blackout_downcounter;
  //! Chips alignment.
  janus_chips_alignment_t chips_alignment;
  //! Chips alignment FIFO.
  janus_utils_fifo_t chips_alignment_fifo;
  //! Offset to apply when chips alignment read baseband samples.
  unsigned chips_alignment_offset;
  //! 32 chip preamble detection: GO CFAR.
  janus_go_cfar_t go_cfar;
  //! Maximum detection time
  janus_real_t maximum_detection_time;
  //! Detection rssi value
  janus_uint8_t rssi;
  //! Detection SNR
  janus_real_t dsnr;
  //! Spectrogram enable.
  janus_uint8_t spectrogram_enable;
  //! Doppler estimator enable.
  janus_uint8_t doppler_enable;
  //! Doppler estimator.
  janus_doppler_t doppler;
  //! Doppler frequency dialation factor.
  janus_real_t gamma;
  //! Estimated Doppler speed [m/s].
  janus_real_t speed;
  //! Chip size in samples.
  janus_real_t chip_size;
  //! Demodulator.
  janus_demodulator_t dmod;
  //! Demodulator: output buffer.
  janus_uint8_t* dmod_bfr;
  //! Demodulator: index of last demodulated chip.
  unsigned dmod_chip;
  //! Demodulator: number of chips to demodulate.
  unsigned dmod_chip_count;
  //! Bit probabilities index.
  unsigned dmod_bit_prob_index;
  //! Deinterleaver: output buffer.
  janus_uint8_t* bfr_dlvr;
  //! Input parameter set.
  janus_pset_t pset;
  //! Input stream.
  janus_istream_t stream;
  //! Current decoder state.
  janus_rx_execution_state_t state;
  //! Detection threshold
  janus_real_t detection_threshold;
  //! Return after demodulation: enabled/disabled.
  janus_uint8_t rx_once;
  //! Assume, the provided signal is already synchronized on the first chip
  janus_uint8_t skip_detection;
  //! Signal offset of the first symbol to be demodulated
  janus_uint32_t detected_offset;
  //! Doppler value
  janus_real_t detected_doppler;
};

static unsigned
rx_bband_fifo_skip(janus_rx_t rx, unsigned samples)
{
  // Skip 'samples' from bband_fifo.
  samples = janus_utils_fifo_skip(rx->bband_fifo, samples * sizeof(janus_complex_t)) / sizeof(janus_complex_t);

  // Update absolute baseband samples counter.
  rx->bband_time_counter += samples;

  return samples;
}

static int
rx_data(janus_rx_t rx, janus_packet_t packet, janus_rx_state_t state)
{
  unsigned read_size = 0;
  unsigned bit_count;
  unsigned byte_count;

  while ((read_size = janus_demodulator_execute(rx->dmod, rx->bband_fifo, rx->dmod_bfr + rx->dmod_chip, (state) ? state->bit_prob + rx->dmod_bit_prob_index : 0)) != 0)
  {
    rx->bband_time_counter += read_size;
    if (state && state->bit_prob_size != -1)
      ++rx->dmod_bit_prob_index;

    // Continue demodulation.
    if (rx->dmod_chip < (rx->dmod_chip_count - 1))
    {
      ++rx->dmod_chip;
      continue;
    }

    // Start decoding process.
    bit_count = (rx->dmod_chip_count / JANUS_ALPHABET_SIZE) - JANUS_CONV_MEM_SIZE_BITS;
    byte_count = bit_count / 8;
    janus_deinterleave(rx->dmod_bfr, rx->dmod_chip_count, rx->bfr_dlvr);

    {
      // Compute traceback length.
      unsigned tbk_len = DEFAULT_VITERBI_TBK_LEN;
      if ((rx->dmod_chip_count / JANUS_ALPHABET_SIZE) < tbk_len)
        tbk_len = rx->dmod_chip_count / JANUS_ALPHABET_SIZE;

      // Decode and convert bits to bytes.
      janus_viterbi_execute(rx->viterbi, rx->bfr_dlvr, rx->dmod_chip_count, rx->viterbi_out_bits, tbk_len);
    }

    janus_utils_bin2dec(rx->viterbi_out_bits, bit_count, rx->viterbi_out_bytes);

    if (rx->state == STATE_PACKET)
    {
      janus_packet_set_bytes(packet, rx->viterbi_out_bytes);

      if (janus_packet_get_crc_validity(packet))
      { 
        janus_packet_base_decode(packet);
        janus_packet_decode_application_data(packet);

        if (janus_packet_get_cargo_size(packet) == 0)
        {
          janus_uint8_t reservation_repeat_flag;
          double tx_interval = janus_packet_get_tx_interval(packet, &reservation_repeat_flag);
          if (reservation_repeat_flag == 0)
          {
            janus_uint64_t reservation_counter;
            janus_rx_reset(rx);
            reservation_counter = (janus_uint64_t)(rx->maximum_detection_time + tx_interval) * rx->bband_fs;
            rx->blackout_downcounter = (unsigned)(reservation_counter - rx->bband_time_counter);
            fprintf(stderr, "Reservation start time %f\n", rx->maximum_detection_time);
            fprintf(stderr, "Reservation end time   %f\n", rx->maximum_detection_time + tx_interval);
          }

          rx->state = STATE_BLACKOUT;
          return 1;
        }

        rx->dmod_chip = 0;
        rx->dmod_chip_count = ((janus_packet_get_cargo_size(packet) * 8) + JANUS_CONV_MEM_SIZE_BITS) * JANUS_ALPHABET_SIZE;

        if (state && state->bit_prob_size != -1)
        {
          state->bit_prob_size = rx->dmod_bit_prob_index + rx->dmod_chip_count;
          state->bit_prob = JANUS_UTILS_MEMORY_REALLOC(state->bit_prob, janus_real_t, state->bit_prob_size);
        }

        rx->state = STATE_PACKET_CARGO;
        return 0;
      }
      // Dump.
      // Incorrect packet only if verbosity >= 3
      if (rx->verbose >= 3  ||
          (rx->verbose > 0 &&
           (janus_packet_get_validity(packet) &&
            janus_packet_get_cargo_error(packet) == 0)))
      {
        fprintf(stderr, "-> Invalid Packet CRC\n");

        state->pset_id = rx->pset->id;
        state->pset_name = rx->pset->name;
        state->cfreq = rx->pset->cfreq;
        state->bwidth = rx->pset->abwidth;
        state->chip_frq = rx->pset->chip_frq;
        state->chip_dur = rx->pset->chip_dur;

        state->after = janus_rx_get_detection_time(rx);
        state->gamma = janus_rx_get_gamma(rx);
        state->speed = janus_rx_get_speed(rx);

        state->rssi  = janus_rx_get_rssi(rx);
        state->snr   = janus_rx_get_snr(rx);

        janus_rx_state_dump(state);
        janus_packet_dump(packet);
        if (rx->rx_once && rx->skip_detection)
            exit(1);
      }

      janus_packet_reset(packet);
      janus_rx_reset(rx);
      return 0;
    }
    else
    {
      janus_packet_set_cargo(packet, rx->viterbi_out_bytes, byte_count);

      int err = janus_packet_decode_cargo(packet);
      
      if (err == 0)
      {
        janus_packet_set_validity(packet, 2);
      }
      else
      {
        janus_packet_set_cargo_error(packet, err);
        fprintf(stderr, "Error decoding cargo (error code %d)\n", err);
      }
      
      rx->state = STATE_BLACKOUT;
      return 1;
    }
  }

  return 0;
}

static int
compare(const void* a, const void* b)
{
  janus_real_t ra = *(janus_real_t*)a;
  janus_real_t rb = *(janus_real_t*)b;
  return (ra > rb) - (ra < rb);
}

janus_rx_t
janus_rx_new(janus_pset_t pset, janus_parameters_t params, janus_istream_t stream, janus_uint8_t verbose)
{
  janus_rx_t rx = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_rx, 1);
  const unsigned n_chips = 32;
  janus_real_t* freq_sec = JANUS_UTILS_MEMORY_ALLOCA(janus_real_t, n_chips);
  unsigned* c_order = JANUS_UTILS_MEMORY_ALLOCA(unsigned, n_chips);
  janus_real_t max_speed = (params->doppler_correction == 0) ? JANUS_REAL_CONST(0.0) : params->doppler_max_speed;
  unsigned channel_spread = 64 * JANUS_PREAMBLE_CHIP_OVERSAMPLING;

  // Set basic variables.
  rx->verbose = verbose;
  rx->pset = pset;
  rx->stream = stream;
  rx->doppler_enable = params->doppler_correction;
  rx->spectrogram_enable = params->compute_channel_spectrogram;
  rx->rx_once = params->rx_once;
  rx->skip_detection = params->skip_detection;
  rx->detected_offset = params->detected_offset;
  rx->detected_doppler = params->detected_doppler;

  // Viterbi decoder.
  rx->viterbi = janus_viterbi_new(janus_trellis_default(), 8, DEFAULT_VITERBI_TBK_LEN);
  rx->viterbi_out_bits = JANUS_UTILS_MEMORY_NEW_ZERO(janus_uint8_t, JANUS_MIN_PKT_SIZE_BITS + JANUS_MAX_PKT_CARGO_SIZE_BITS);
  rx->viterbi_out_bytes = JANUS_UTILS_MEMORY_NEW_ZERO(janus_uint8_t, JANUS_MIN_PKT_SIZE + JANUS_MAX_PKT_CARGO_SIZE);

  // Digital down converter.
  rx->bband_fs = janus_istream_get_output_fs(stream);

  // Chips alignment.
  {
    janus_real_t ofslots[26];
    unsigned* f_order = JANUS_UTILS_MEMORY_ALLOCA(unsigned, n_chips);
    janus_real_t* freq_vector;
    unsigned freq_vector_size;
    unsigned i, j;
    int o;

    freq_vector_size = JANUS_CHIP_FRQ_COUNT * 2;

    for (o = 0; o < (int)freq_vector_size; o++)
    {
      ofslots[o] = (-(JANUS_FLOOR(JANUS_CHIP_FRQ_COUNT / 2.0) * JANUS_ALPHABET_SIZE + 1.0) + o) * (int)pset->chip_frq;
    }

    for (i = 0; i < n_chips; i++)
    {
      o = ((pset->c32_sequence) & (1 << (31 - i))) ? 1 : 0;
      f_order[i] = janus_hop_index(i, pset->primitive.alpha, pset->primitive.q) * JANUS_ALPHABET_SIZE + o;
    }

    for (i = 0; i < n_chips; i++)
    {
      freq_sec[i] = ofslots[f_order[i]];
    }

    freq_vector = JANUS_UTILS_MEMORY_ALLOCA(janus_real_t, freq_vector_size);

    if (rx->spectrogram_enable)
    {
      for (i = 0; i != freq_vector_size; i++)
      {
        freq_vector[i] = ofslots[i];
      }

      memcpy(c_order, f_order, n_chips * sizeof(unsigned));
    }
    else
    {
      janus_real_t* freq_sec_sorted = JANUS_UTILS_MEMORY_ALLOCA(janus_real_t, n_chips);
      memcpy(freq_sec_sorted, freq_sec, n_chips * sizeof(janus_real_t));
      qsort(freq_sec_sorted, n_chips, sizeof(janus_real_t), compare);

      freq_vector_size = 1;
      for (i = 1; i < n_chips; i++)
      {
        if (freq_sec_sorted[i] != freq_sec_sorted[i - 1])
        {
          freq_vector_size++;
        }
      }

      freq_vector[0] = freq_sec_sorted[0];
      j = 1;
      for (i = 1; i < n_chips; i++)
      {
        if (freq_sec_sorted[i] != freq_sec_sorted[i - 1])
        {
          freq_vector[j++] = freq_sec_sorted[i];
        }
      }
      JANUS_UTILS_MEMORY_ALLOCA_FREE(freq_sec_sorted);

      for (i = 0; i < n_chips; i++)
      {
        for (j = 0; j < freq_vector_size; j++)
        {
          if (freq_sec[i] == freq_vector[j])
          {
            c_order[i] = j;
            break;
          }
        }
      }
    }

    rx->chips_alignment = janus_chips_alignment_new(rx->bband_fs, pset->chip_dur, freq_vector_size, freq_vector, n_chips, c_order, max_speed);
    rx->chips_alignment_fifo = janus_utils_fifo_new(BBAND_FIFO_SIZE * JANUS_PREAMBLE_CHIP_OVERSAMPLING * sizeof(janus_real_t));

    JANUS_UTILS_MEMORY_ALLOCA_FREE(f_order);
    JANUS_UTILS_MEMORY_ALLOCA_FREE(freq_vector);
  }
  rx->chip_size = pset->chip_dur * rx->bband_fs;

  rx->dmod_chip_count = (JANUS_MIN_PKT_SIZE_BITS + JANUS_CONV_MEM_SIZE_BITS) * JANUS_ALPHABET_SIZE;

  // GO-CFAR
  {
    janus_real_t mov_avg_time = JANUS_REAL_CONST(0.150);
    janus_real_t mov_avg = JANUS_FMAX(mov_avg_time, 26 * pset->chip_dur);
    unsigned step_length = (unsigned)JANUS_FLOOR(JANUS_PREAMBLE_CHIP_OVERSAMPLING * mov_avg / pset->chip_dur);
    unsigned guard_size = JANUS_PREAMBLE_CHIP_OVERSAMPLING * 2;
    unsigned window_size = step_length * 2 + guard_size;
    unsigned stream_size = (unsigned)JANUS_CEIL(stream->out_size * JANUS_PREAMBLE_CHIP_OVERSAMPLING / rx->chip_size);
    janus_real_t window_correction = JANUS_REAL_CONST(0.1538) * JANUS_FMIM((n_chips + rx->dmod_chip_count) / JANUS_ROUND(step_length / 4), 1);

    rx->go_cfar = janus_go_cfar_new(window_size, guard_size, window_correction, stream_size, channel_spread, JANUS_PREAMBLE_CHIP_OVERSAMPLING);
    rx->maximum_detection_time = -1.0;
    janus_rx_set_threshold(rx, params->detection_threshold);
  }

  // Doppler estimator.
  rx->gamma = JANUS_REAL_CONST(1.0);
  rx->speed = JANUS_REAL_CONST(0.0);
  rx->doppler = janus_doppler_new(rx->doppler_enable, rx->pset, freq_sec, n_chips, rx->bband_fs, max_speed);

  // Demodulator.
  rx->dmod = janus_demodulator_new(pset, rx->bband_fs, janus_doppler_get_min_gamma(rx->doppler));
  rx->dmod_bfr = JANUS_UTILS_MEMORY_NEW_ZERO(janus_uint8_t, 2 * 8 * 4096);
  rx->dmod_bit_prob_index = 0;

  // Deinterleaver.
  rx->bfr_dlvr = JANUS_UTILS_MEMORY_NEW_ZERO(janus_uint8_t, 2 * 8 * 4096);

  // Baseband fifo.
  {
    unsigned store_size = (unsigned)JANUS_CEIL((n_chips + janus_udiv_ceil(channel_spread + janus_go_cfar_get_window_size(rx->go_cfar), JANUS_PREAMBLE_CHIP_OVERSAMPLING)) * rx->chip_size);
    rx->bband_fifo = janus_utils_fifo_new((BBAND_FIFO_SIZE + store_size) * sizeof(janus_complex_t));
    rx->bband_time_counter = 0;
  }

  janus_rx_reset(rx);

  JANUS_UTILS_MEMORY_ALLOCA_FREE(freq_sec);
  JANUS_UTILS_MEMORY_ALLOCA_FREE(c_order);

  return rx;
}

void
janus_rx_free(janus_rx_t rx)
{
  janus_viterbi_free(rx->viterbi);
  JANUS_UTILS_MEMORY_FREE(rx->viterbi_out_bits);
  JANUS_UTILS_MEMORY_FREE(rx->viterbi_out_bytes);
  janus_doppler_free(rx->doppler);
  janus_demodulator_free(rx->dmod);
  JANUS_UTILS_MEMORY_FREE(rx->dmod_bfr);
  janus_chips_alignment_free(rx->chips_alignment);
  janus_go_cfar_free(rx->go_cfar);
  janus_utils_fifo_free(rx->chips_alignment_fifo);
  janus_utils_fifo_free(rx->bband_fifo);
  JANUS_UTILS_MEMORY_FREE(rx->bfr_dlvr);

  if (rx->doppler_enable && rx->stream->ddc == 0)
    JANUS_FFTW_PREFIX(cleanup)();

  JANUS_UTILS_MEMORY_FREE(rx);
}

int
janus_rx_execute(janus_rx_t rx, janus_packet_t packet, janus_rx_state_t state)
{
//  printf("ENTER in rx_execute exit\n");
//  exit(1);

  int skip_size;
  janus_complex_t* bband = NULL;
  int rv = janus_istream_read(rx->stream, &bband);

  if (rx->skip_detection && rx->state < STATE_PACKET) {
      skip_size = (rx->detected_offset + 128 /* FIR delay */) / 4;

      janus_utils_fifo_put(rx->bband_fifo, bband, rv * sizeof(janus_complex_t));
      /* printf("put: %d\n", janus_utils_fifo_get_size(rx->bband_fifo)); */
      if (janus_utils_fifo_get_size(rx->bband_fifo) < skip_size * 16) {
          ;
      } else {
          /* rx->gamma = JANUS_REAL_CONST(1.0); */
          /* rx->speed = JANUS_REAL_CONST(0.0); */
          rx->gamma = JANUS_REAL_CONST(rx->detected_doppler);
          rx->speed = JANUS_REAL_CONST(1540.0) * (1 - rx->gamma);

          printf("gamma = %f, speed = %f, offset = %d\n", rx->gamma, rx->speed, rx->detected_offset);
          
          rx_bband_fifo_skip(rx, (unsigned)skip_size);
          
          janus_demodulator_set_cfactor(rx->dmod, rx->gamma);
          janus_demodulator_set_index(rx->dmod, janus_chips_alignment_get_preamble_size(rx->chips_alignment));
          
          if (state && state->bit_prob_size != -1)
          {
              // Setting bit probabilities buffer, size and index.
              state->bit_prob_size = rx->dmod_chip_count;
              state->bit_prob = JANUS_UTILS_MEMORY_REALLOC(state->bit_prob, janus_real_t, state->bit_prob_size);
          }
          rx->state = STATE_PACKET;
      }
      return 0;
  }
  
  
  if (rv <= 0)
  {
    if (rv == JANUS_ERROR_OVERRUN)
    {
      return JANUS_ERROR_OVERRUN;
    }
    return JANUS_ERROR_STREAM;
  }

  janus_utils_fifo_put(rx->bband_fifo, bband, rv * sizeof(janus_complex_t));

  if (janus_utils_fifo_is_empty(rx->bband_fifo))
    return JANUS_ERROR_EOS;

#if 0
  for (unsigned i = 0; i < rv; i++)
    printf("%+.15f %+.15f\n", bband[i][0], bband[i][1]);
#endif

  // Blackout.
  if (rx->state == STATE_BLACKOUT)
  {
    if (rx->rx_once)
        return JANUS_ERROR_STREAM;

    rx->blackout_downcounter -= rx_bband_fifo_skip(rx, rx->blackout_downcounter);
    if (rx->blackout_downcounter == 0)
    {
      janus_rx_reset(rx);
      return 0;
    }
  }

  // Search 32 chip preamble.
  if (rx->state == STATE_32_CHIP_PREAMBLE)
  {
    // Chips alignment.
    rx->chips_alignment_offset += janus_chips_alignment_execute(rx->chips_alignment, rx->bband_fifo, rx->chips_alignment_offset, rx->chips_alignment_fifo);

    // GO-CFAR
    rv = (int)janus_go_cfar_execute(rx->go_cfar, rx->chips_alignment_fifo);

    if (janus_go_cfar_detection_competed(rx->go_cfar) == 0)
    {
      skip_size = (int)((rx->bband_cfar_init_counter + ((janus_go_cfar_get_counter(rx->go_cfar) + 1) * rx->chip_size / JANUS_PREAMBLE_CHIP_OVERSAMPLING)) -
                        rx->chip_size -
                        rx->bband_time_counter);
    }
    else
    {
      skip_size = (int)((rx->bband_cfar_init_counter +
                        ((janus_go_cfar_get_counter(rx->go_cfar) + 1.0 + janus_go_cfar_get_detection_offset(rx->go_cfar)) * rx->chip_size / JANUS_PREAMBLE_CHIP_OVERSAMPLING)) +
                        (janus_chips_alignment_get_align_delay(rx->chips_alignment) * rx->bband_fs) -
                        rx->bband_time_counter);
    }

    if (skip_size > 0)
    {
      rx_bband_fifo_skip(rx, (unsigned)skip_size);
      rx->chips_alignment_offset -= skip_size;
    }

    if (janus_go_cfar_detection_competed(rx->go_cfar) > 0)
    {
      janus_real_t peak_detection_energy;

      rx->maximum_detection_time = (janus_real_t)rx->bband_time_counter / rx->bband_fs;
      if (rx->stream->ddc)
      {
        rx->maximum_detection_time -= JANUS_REAL_CONST(128.0) / rx->stream->fs;
      }

      if ((peak_detection_energy =
           janus_go_cfar_get_detection_peak_energy(rx->go_cfar)) > 0)
      {
        janus_real_t frssi = JANUS_REAL_CONST(255.0)
          + JANUS_REAL_CONST(7.997548596232297)
          * JANUS_LOG(peak_detection_energy);

#if 1
        printf("frssi = %.3f\n", frssi);
#endif // 1

        if (frssi > JANUS_REAL_CONST(255.0))
        {
          rx->rssi = 255;
        }
        else
        {
          if (frssi < 0)
          {
            rx->rssi = 0;
          }
          else
          {
            rx->rssi = JANUS_ROUND(frssi);
          }
        }

        rx->dsnr = JANUS_LOG(peak_detection_energy /
                             janus_go_cfar_get_detection_background_energy(rx->go_cfar))
          * JANUS_REAL_CONST(4.342944819032518);
      }

      rx->state = STATE_DOPPLER_ESTIMATION;

      return 0; // Comment to disable detection notification on the receiver
    }
  }

  if (rx->state == STATE_DOPPLER_ESTIMATION)
  {
    rx->gamma = JANUS_REAL_CONST(1.0);
    rx->speed = JANUS_REAL_CONST(0.0);
    skip_size = janus_doppler_execute(rx->doppler, rx->bband_fifo, &(rx->gamma), &(rx->speed));

    if (skip_size)
    {
      rx_bband_fifo_skip(rx, (unsigned)skip_size);

      janus_demodulator_set_cfactor(rx->dmod, rx->gamma);
      janus_demodulator_set_index(rx->dmod, janus_chips_alignment_get_preamble_size(rx->chips_alignment));

      if (state && state->bit_prob_size != -1)
      {
        // Setting bit probabilities buffer, size and index.
        state->bit_prob_size = rx->dmod_chip_count;
        state->bit_prob = JANUS_UTILS_MEMORY_REALLOC(state->bit_prob, janus_real_t, state->bit_prob_size);
      }

      rx->state = STATE_PACKET;
    }
  }

  if (rx->state == STATE_PACKET || rx->state == STATE_PACKET_CARGO)
  {
    if (rx_data(rx, packet, state) > 0)
    {
      if (state)
      {
        state->pset_id = rx->pset->id;
        state->pset_name = rx->pset->name;
        state->cfreq = rx->pset->cfreq;
        state->bwidth = rx->pset->abwidth;
        state->chip_frq = rx->pset->chip_frq;
        state->chip_dur = rx->pset->chip_dur;

        state->after = janus_rx_get_detection_time(rx);
        state->gamma = janus_rx_get_gamma(rx);
        state->speed = janus_rx_get_speed(rx);

        state->rssi  = janus_rx_get_rssi(rx);
        state->snr   = janus_rx_get_snr(rx);
      }
      return 1;
    }
  }

  return 0;
}

void
janus_rx_reset(janus_rx_t rx)
{
  rx->state = STATE_32_CHIP_PREAMBLE;
  rx->dmod_chip = 0;
  rx->dmod_chip_count = (JANUS_MIN_PKT_SIZE_BITS + JANUS_CONV_MEM_SIZE_BITS) * JANUS_ALPHABET_SIZE;
  rx->dmod_bit_prob_index = 0;

  janus_utils_fifo_reset(rx->chips_alignment_fifo);
  rx->chips_alignment_offset = 0;

  rx->bband_cfar_init_counter = rx->bband_time_counter;
  rx->blackout_downcounter = (unsigned)(rx->bband_fs * JANUS_BLACKOUT_TIME);

  janus_chips_alignment_reset(rx->chips_alignment);
  janus_go_cfar_reset(rx->go_cfar);
  janus_doppler_reset(rx->doppler);
  janus_demodulator_reset(rx->dmod);
}

int
janus_rx_has_detected(janus_rx_t rx)
{
  return janus_go_cfar_has_detected(rx->go_cfar) ||
         rx->state == STATE_DOPPLER_ESTIMATION ||
         rx->state == STATE_PACKET ||
         rx->state == STATE_PACKET_CARGO;
}

int
janus_rx_is_decoding(janus_rx_t rx)
{
  return rx->state == STATE_DOPPLER_ESTIMATION ||
         rx->state == STATE_PACKET ||
         rx->state == STATE_PACKET_CARGO;
}

janus_real_t
janus_rx_get_first_detection_time(janus_rx_t rx)
{
  janus_real_t first_detection_time;

  janus_uint64_t first_detection_counter = janus_go_cfar_get_first_detection(rx->go_cfar) + 1;

  first_detection_time = (rx->bband_cfar_init_counter + first_detection_counter * rx->chip_size / JANUS_PREAMBLE_CHIP_OVERSAMPLING) / rx->bband_fs +
                         janus_chips_alignment_get_align_delay(rx->chips_alignment);
  if (rx->stream->ddc)
  {
    first_detection_time -= JANUS_REAL_CONST(128.0) / rx->stream->fs;
  }

  return first_detection_time;
}

janus_real_t
janus_rx_get_detection_time(janus_rx_t rx)
{
  return rx->maximum_detection_time;
}

janus_real_t
janus_rx_get_gamma(janus_rx_t rx)
{
  return rx->gamma;
}

janus_real_t
janus_rx_get_speed(janus_rx_t rx)
{
  return rx->speed;
}

void
janus_rx_get_frequencies(janus_rx_t rx, unsigned* freq_vector_size, janus_real_t** freq_vector)
{
  janus_chips_alignment_get_frequencies(rx->chips_alignment, freq_vector_size, freq_vector);
}

int
janus_rx_get_channel_spectrogram(janus_rx_t rx, janus_real_t** spectrogram, unsigned* valid, janus_real_t* last_time, janus_real_t* step_time)
{
  if ((rx->spectrogram_enable == 1) && (rx->state == STATE_32_CHIP_PREAMBLE))
  {
    *valid = janus_chips_alignment_get_spectrogram(rx->chips_alignment, spectrogram);

    *step_time = (rx->chip_size / JANUS_PREAMBLE_CHIP_OVERSAMPLING) / rx->bband_fs;

    *last_time = (rx->bband_cfar_init_counter + janus_chips_alignment_get_bb_counter(rx->chips_alignment) -
                 janus_chips_alignment_get_goertzel_windows_size(rx->chips_alignment)) / rx->bband_fs +
                 janus_chips_alignment_get_align_delay(rx->chips_alignment);

    if (rx->stream->ddc)
    {
      *last_time -= JANUS_REAL_CONST(128.0) / rx->stream->fs;
    }

    return 1;
  }
  else
  {
    return 0;
  }
}

janus_uint8_t
janus_rx_get_rssi(janus_rx_t rx)
{
  return rx->rssi;
}

janus_real_t
janus_rx_get_snr(janus_rx_t rx)
{
  return rx->dsnr;
}

void
janus_rx_set_threshold(janus_rx_t rx, janus_real_t detection_threshold)
{
  rx->detection_threshold = detection_threshold;
  janus_go_cfar_set_threshold(rx->go_cfar, detection_threshold);
}

