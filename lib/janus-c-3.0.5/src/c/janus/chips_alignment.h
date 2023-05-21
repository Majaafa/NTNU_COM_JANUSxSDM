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

#ifndef JANUS_CHIPS_ALIGNMENT_H_INCLUDED_
#define JANUS_CHIPS_ALIGNMENT_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/complex.h>
#include <janus/export.h>
#include <janus/utils/fifo.h>

//! Chips alignment algorithm opaque object.
typedef struct janus_chips_alignment* janus_chips_alignment_t;

//! Create a chips alignment algorithm object.
//! @param fs sampling frequency.
//! @param chip_duration chip duration.
//! @param freq_vector_size number of frequencies.
//! @param freq_vector frequencies.
//! @param n_chips preamble size (number of chips in preamble).
//! @param c_order preamble pattern (order of frequencies within preamble).
//! @param max_speed_ms doppler max speed (in meters/sec).
//! @return chips alignment algorithm object.
JANUS_EXPORT janus_chips_alignment_t
janus_chips_alignment_new(unsigned fs, janus_real_t chip_duration, unsigned freq_vector_size, janus_real_t freq_vector[], unsigned n_chips, unsigned c_order[], janus_real_t max_speed_ms);

//! Free chips alignment algorithm object.
//! @param chips_alignment chips alignment algorithm object.
JANUS_EXPORT void
janus_chips_alignment_free(janus_chips_alignment_t chips_alignment);

//! Reset chips alignment algorithm object.
//! @param chips_alignment chips alignment algorithm object.
JANUS_EXPORT void
janus_chips_alignment_reset(janus_chips_alignment_t chips_alignment);

//! Execute chips alignment algorithm.
//! @param chips_alignment chips alignment algorithm object.
//! @param ififo input FIFO with complex baseband signal.
//! @param ififo_offset offset to apply when read baseband samples.
//! @param ofifo output FIFO with result of chips alignment algorithm (real numbers).
JANUS_EXPORT unsigned
janus_chips_alignment_execute(janus_chips_alignment_t chips_alignment, janus_utils_fifo_t ififo, unsigned ififo_offset, janus_utils_fifo_t ofifo);

//! Get the goertzel frequencies.
//! @param chips_alignment chips alignment algorithm object.
//! @param goertzel frequencies vector.
//! @param goertzel frequencies vector size.
JANUS_EXPORT void
janus_chips_alignment_get_frequencies(janus_chips_alignment_t chips_alignment, unsigned* freq_vector_size, janus_real_t** freq_vector);

//! Get the preamble size (number of chips in preamble).
//! @param chips_alignment chips alignment algorithm object.
//! @return preamble size (number of chips in preamble).
JANUS_EXPORT unsigned
janus_chips_alignment_get_preamble_size(janus_chips_alignment_t chips_alignment);

//! Get the last used baseband sample counter.
//! @param chips_alignment chips alignment algorithm object.
//! @return last used baseband sample counter.
JANUS_EXPORT janus_hiprecision_t
janus_chips_alignment_get_bb_counter(janus_chips_alignment_t chips_alignment);

//! Get the goertzel windows size.
//! @param chips_alignment chips alignment algorithm object.
//! @return goertzel windows size.
JANUS_EXPORT unsigned
janus_chips_alignment_get_goertzel_windows_size(janus_chips_alignment_t chips_alignment);

//! Get the delay from baseband signal and chip_corr (in seconds).
//! @param chips_alignment chips alignment algorithm object.
//! @return delay.
JANUS_EXPORT janus_real_t
janus_chips_alignment_get_align_delay(janus_chips_alignment_t chips_alignment);

//! Get the last goertzel values.
//! @param chips_alignment chips alignment algorithm object.
//! @param spectrogram matrix pointer 
//! @return the number of valid samples.
JANUS_EXPORT unsigned
janus_chips_alignment_get_spectrogram(janus_chips_alignment_t chips_alignment, janus_real_t** spectrogram);

#endif
