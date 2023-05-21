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
// Author: Ricardo Martins                                                *
//*************************************************************************

// ISO C headers.
#include <stdlib.h>
#include <math.h>
#include <string.h>

// JANUS headers.
#include <janus/defaults.h>
#include <janus/dump.h>
#include <janus/wake_up_tones.h>
#include <janus/convolve.h>
#include <janus/interleave.h>
#include <janus/modulator.h>
#include <janus/packet.h>
#include <janus/tx.h>
#include <janus/trellis.h>
#include <janus/utils/memory.h>
#include <janus/utils/dec2bin.h>

struct janus_tx
{
  //! Parameter set.
  janus_pset_t pset;
  //! Output stream.
  janus_ostream_t ostream;
  //! Trellis structure.
  janus_trellis_t trellis;
  //! Number of bits in the packet, including convolution memory.
  unsigned pkt_bits;
  //! Number of packet chips.
  unsigned pkt_chip_count;
  //! Number of silence samples (padding).
  unsigned pad_size;
  //! True to transmit wake up tones.
  janus_uint8_t wut;
  //! True to pad signal.
  janus_uint8_t pad;
  //! Modulator.
  janus_modulator_t modulator;
};

static int
write_silence(janus_ostream_t ostream, unsigned sample_count)
{
  janus_complex_t tmp = {0};
  unsigned i;
  for (i = 0; i < sample_count; ++i)
  {
    int rv;
    if ((rv = janus_ostream_write(ostream, &tmp, 1)) < 0)
    {
      return rv;
    }
  }
  
  return 0;
}

janus_tx_t
janus_tx_new(janus_pset_t pset, janus_ostream_t ostream, janus_uint8_t verbose)
{
  janus_tx_t tx = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_tx, 1);
  tx->pset = pset;
  tx->ostream = ostream;
  tx->trellis = janus_trellis_default();
  tx->pkt_bits = JANUS_MIN_PKT_SIZE_BITS + JANUS_CONV_MEM_SIZE_BITS;
  tx->pkt_chip_count = JANUS_ALPHABET_SIZE * tx->pkt_bits;
  tx->modulator = janus_modulator_new(pset, ostream->fs);
  janus_tx_set_padding(tx, 1);
  janus_tx_set_wake_up_tones(tx, 1);

  return tx;
}

void
janus_tx_free(janus_tx_t tx)
{
  if (tx != 0)
  {
    janus_modulator_free(tx->modulator);
  }
  JANUS_UTILS_MEMORY_FREE(tx);
}

int
janus_tx_execute(janus_tx_t tx, janus_packet_t packet, janus_tx_state_t state)
{
  // Compute lengths.
  unsigned dat_len = janus_packet_get_cargo_size(packet);
  unsigned dat_bits = (dat_len == 0) ? 0 : ((dat_len + JANUS_CONV_MEM_SIZE) * 8);
  unsigned dat_chip_count = dat_bits * JANUS_ALPHABET_SIZE;
  unsigned chip_count = JANUS_PREAMBLE_CHIP_COUNT + tx->pkt_chip_count + dat_chip_count;
  unsigned sample_count = 0;
  janus_uint8_t* coded = NULL;
  janus_uint8_t* coded_ptr = NULL;
  janus_uint8_t* pkt = NULL;
  janus_uint8_t* pkt_conv = NULL;
  janus_uint8_t* dat = NULL;
  janus_uint8_t* dat_conv = NULL;
  janus_trellis_t trellis = janus_trellis_default();

  // Transmit prologue silence.
  if (tx->pad)
  {
    int rv;
    if ((rv = write_silence(tx->ostream, tx->pad_size)) < 0)
    {
      return rv;
    }
    sample_count += tx->pad_size;
  }

  // Transmit optional wake-up tones.
  if (tx->wut)
  {
    int rv;
    if ((rv = janus_wake_up_tones(tx->ostream, tx->pset->ubwidth, tx->pset->chip_dur)) < 0)
    {
      return rv;
    }
    sample_count += rv;
    if ((rv = write_silence(tx->ostream, (unsigned)(JANUS_WUT_GAP * tx->ostream->fs))) < 0)
    {
      return rv;
    }
    sample_count += tx->pad_size;
  }

  // Prepare output memory.
  coded = JANUS_UTILS_MEMORY_NEW(janus_uint8_t, chip_count);
  coded_ptr = coded;

  // Initialize preamble chip sequence.
  coded_ptr += JANUS_PREAMBLE_CHIP_COUNT;
  janus_utils_dec2bin_byte(tx->pset->c32_sequence >> 24, coded);
  janus_utils_dec2bin_byte(tx->pset->c32_sequence >> 16, coded + 8);
  janus_utils_dec2bin_byte(tx->pset->c32_sequence >> 8, coded + 16);
  janus_utils_dec2bin_byte(tx->pset->c32_sequence >> 0, coded + 24);

  // Prepare packet.
  janus_packet_set_crc(packet);

  // Process packet.
  pkt = JANUS_UTILS_MEMORY_NEW_ZERO(janus_uint8_t, tx->pkt_bits);
  janus_utils_dec2bin(janus_packet_get_bytes(packet), JANUS_MIN_PKT_SIZE, pkt);
  pkt_conv = JANUS_UTILS_MEMORY_NEW_ZERO(janus_uint8_t, tx->pkt_chip_count);
  janus_convolve(trellis, pkt, tx->pkt_bits, pkt_conv);
  JANUS_UTILS_MEMORY_FREE(pkt);
  janus_interleave(pkt_conv, tx->pkt_chip_count, coded_ptr);
  JANUS_UTILS_MEMORY_FREE(pkt_conv);
  coded_ptr += tx->pkt_chip_count;

  // Process optional cargo.
  if (dat_bits)
  {
    dat = JANUS_UTILS_MEMORY_NEW_ZERO(janus_uint8_t, dat_bits);
    janus_utils_dec2bin(janus_packet_get_cargo(packet), dat_len, dat);
    dat_conv = JANUS_UTILS_MEMORY_NEW_ZERO(janus_uint8_t, dat_chip_count);
    janus_convolve(trellis, dat, dat_bits, dat_conv);
    JANUS_UTILS_MEMORY_FREE(dat);
    janus_interleave(dat_conv, dat_chip_count, coded_ptr);
    JANUS_UTILS_MEMORY_FREE(dat_conv);
  }

  // Modulate and transmit data.
  {
    janus_complex_t* out = NULL;
    unsigned i;
    for (i = 0; i < chip_count; ++i)
    {
      int err = 0;
      unsigned rv = janus_modulator_execute(tx->modulator, coded[i], &out);
      if ((err = janus_ostream_write(tx->ostream, out, rv)) < 0)
      {
        JANUS_UTILS_MEMORY_FREE(coded);
        return err;
      }
      sample_count += rv;
    }
  }

  // Transmit epilogue silence.
  if (tx->pad)
  {
    int rv;
    if ((rv = write_silence(tx->ostream, tx->pad_size)) < 0)
    {
      return rv;
    }
    sample_count += tx->pad_size;
  }

  // Pad signal if needed.
  if (tx->ostream->multiple > 1)
  {
    unsigned rem = sample_count % tx->ostream->multiple;
    if (rem != 0)
    {
      int rv;
      if ((rv = write_silence(tx->ostream, tx->ostream->multiple - rem)) < 0)
      {
        return rv;
      }
    }
  }

  // Reset output stream.
  janus_ostream_reset(tx->ostream);

  // Reset modulator.
  janus_modulator_reset(tx->modulator);
  
  if (state)
  {
    state->pset_id = tx->pset->id;
    state->pset_name = tx->pset->name;
    state->cfreq = tx->pset->cfreq;
    state->bwidth = tx->pset->abwidth;
    state->chip_frq = tx->pset->chip_frq;
    state->chip_dur = tx->pset->chip_dur;

    if (state->coded_symbols_enabled)
    {
      state->coded_symbols_size = chip_count - JANUS_PREAMBLE_CHIP_COUNT;
      state->coded_symbols = JANUS_UTILS_MEMORY_REALLOC(state->coded_symbols, janus_uint8_t, state->coded_symbols_size);
      memcpy(state->coded_symbols, coded + JANUS_PREAMBLE_CHIP_COUNT, state->coded_symbols_size);
    }
  }

  JANUS_UTILS_MEMORY_FREE(coded);

  return 0;
}

void
janus_tx_set_padding(janus_tx_t tx, janus_uint8_t enabled)
{
  tx->pad = enabled;

  if (enabled)
  {
    janus_real_t a = JANUS_ROUND(tx->pset->chip_dur / tx->ostream->ts);
    janus_real_t b = JANUS_ROUND(tx->pset->chip_dur * tx->ostream->fs);
    tx->pad_size = (unsigned)(JANUS_FMAX(a, b) * JANUS_SILENCE_CHIP_COUNT);
  }
}

void
janus_tx_set_wake_up_tones(janus_tx_t tx, janus_uint8_t enabled)
{
  tx->wut = enabled;
}

void
janus_tx_set_stream_amp(janus_tx_t tx, janus_real_t stream_amp)
{
  janus_ostream_set_amplitude_factor(tx->ostream, stream_amp);
}
