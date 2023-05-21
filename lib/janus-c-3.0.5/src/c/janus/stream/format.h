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

#ifndef JANUS_STREAM_FORMAT_H_INCLUDED_
#define JANUS_STREAM_FORMAT_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/complex.h>
#include <janus/export.h>

//! @ingroup STREAMS
//! @{

//! Supported formats.
typedef enum
{
  //! Unknown format.
  JANUS_STREAM_FORMAT_UNKNOWN,
  //! Signed 8 bit PCM.
  JANUS_STREAM_FORMAT_S8,
  //! Signed 10 bit PCM.
  JANUS_STREAM_FORMAT_S10,
  //! Signed 12 bit PCM.
  JANUS_STREAM_FORMAT_S12,
  //! Signed 14 bit PCM.
  JANUS_STREAM_FORMAT_S14,
  //! Signed 16 bit PCM.
  JANUS_STREAM_FORMAT_S16,
  //! Signed 24 bit PCM.
  JANUS_STREAM_FORMAT_S24,
  //! Signed 24 bit PCM in LSB of 32 Bit words.
  JANUS_STREAM_FORMAT_S24_32,
  //! Signed 24 bit PCM on 3 bytes little endian.
  JANUS_STREAM_FORMAT_S24_3LE,
  //! Signed 32 bit PCM.
  JANUS_STREAM_FORMAT_S32,
  //! 32 bit IEEE floating point [-1.0, 1.0].
  JANUS_STREAM_FORMAT_FLOAT,
  //! 64 bit IEEE floating point [-1.0, 1.0].
  JANUS_STREAM_FORMAT_DOUBLE
} janus_stream_format_t;

typedef void (*janus_stream_format_quantizer_t)(const janus_real_t* samples, unsigned sample_count, void* frames, unsigned channel_count, unsigned channel);
typedef void (*janus_stream_format_quantizer_complex_t)(janus_complex_t* samples, unsigned sample_count, void* frames);

typedef void (*janus_stream_format_normalizer_t)(const void* frames, unsigned frame_count, janus_real_t* samples, unsigned channel_count, unsigned channel);
typedef void (*janus_stream_format_normalizer_complex_t)(const void* frames, unsigned frame_count, janus_complex_t* samples);

JANUS_EXPORT const char*
janus_stream_format_get_name(janus_stream_format_t format);

JANUS_EXPORT janus_stream_format_t
janus_stream_format_parse(const char* format);

JANUS_EXPORT unsigned
janus_stream_format_get_sample_size(janus_stream_format_t format);

JANUS_EXPORT unsigned
janus_stream_format_get_sample_size_bits(janus_stream_format_t format);

JANUS_EXPORT janus_stream_format_quantizer_t
janus_stream_format_get_quantizer(janus_stream_format_t format);

JANUS_EXPORT janus_stream_format_quantizer_complex_t
janus_stream_format_get_quantizer_complex(janus_stream_format_t format);

JANUS_EXPORT janus_stream_format_normalizer_t
janus_stream_format_get_normalizer(janus_stream_format_t format);

JANUS_EXPORT janus_stream_format_normalizer_complex_t
janus_stream_format_get_normalizer_complex(janus_stream_format_t format);

//! @}

#endif
