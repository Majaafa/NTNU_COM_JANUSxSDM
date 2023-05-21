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
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

// JANUS headers.
#include <janus/stream/format.h>

typedef unsigned char janus_int24_t[3];

static inline int
int24_to_int(janus_int24_t x)
{
  int tmp = 0;
  struct {signed int x:24;} s;
  memcpy(&tmp, x, 3);
  return s.x = tmp;
}

#define S8_TYPE                    janus_int8_t
#define S8_BITS                    8
#define S8_SCALE                   JANUS_REAL_CONST(128.0)
#define S8_QUANTIZE(from, to)      {to = (S8_TYPE)(from * S8_SCALE);}
#define S8_NORMALIZE(from, to)     {to = (from / S8_SCALE);}
#define S8_ZERO(to)                {to = 0;}

#define S10_TYPE                   janus_int16_t
#define S10_BITS                   10
#define S10_SCALE                  JANUS_REAL_CONST(513.0)
#define S10_QUANTIZE(from, to)     {to = (S10_TYPE)(from * S10_SCALE);}
#define S10_NORMALIZE(from, to)    {to = (from / S10_SCALE);}
#define S10_ZERO(to)               {to = 0;}

#define S12_TYPE                   janus_int16_t
#define S12_BITS                   12
#define S12_SCALE                  JANUS_REAL_CONST(2048.0)
#define S12_QUANTIZE(from, to)     {to = (S12_TYPE)(from * S12_SCALE);}
#define S12_NORMALIZE(from, to)    {to = (from / S12_SCALE);}
#define S12_ZERO(to)               {to = 0;}

#define S14_TYPE                   janus_int16_t
#define S14_BITS                   14
#define S14_SCALE                  JANUS_REAL_CONST(8193.0)
#define S14_QUANTIZE(from, to)     {to = (S14_TYPE)(from * S14_SCALE);}
#define S14_NORMALIZE(from, to)    {to = (from / S14_SCALE);}
#define S14_ZERO(to)               {to = 0;}

#define S16_TYPE                   janus_int16_t
#define S16_BITS                   16
#define S16_SCALE                  JANUS_REAL_CONST(32768.0)
#define S16_QUANTIZE(from, to)     {to = (S16_TYPE)(from * S16_SCALE);}
#define S16_NORMALIZE(from, to)    {to = (from / S16_SCALE);}
#define S16_ZERO(to)               {to = 0;}

#define S24_TYPE                   janus_int24_t
#define S24_BITS                   24
#define S24_SCALE                  JANUS_REAL_CONST(8388608.0)
#define S24_QUANTIZE(from, to)     {int tmp__ = (int)(from * S24_SCALE); memcpy(to, &tmp__, 3);}
#define S24_NORMALIZE(from, to)    {to = int24_to_int(from) / S24_SCALE;}
#define S24_ZERO(to)               {memset(to, 0, 3);}

#define S24_32_TYPE                janus_int32_t
#define S24_32_BITS                24
#define S24_32_SCALE               S24_SCALE
#define S24_32_QUANTIZE(from, to)  {to = (S24_32_TYPE)(from * S24_32_SCALE);}
#define S24_32_NORMALIZE(from, to) {to = (from / S24_32_SCALE);}
#define S24_32_ZERO(to)            {to = 0;}

#define S24_3LE_TYPE               janus_int24_t
#define S24_3LE_BITS               24
#define S24_3LE_SCALE              S24_SCALE
#define S24_3LE_QUANTIZE(from, to) {int tmp__ = (int)(from * S24_SCALE); memcpy(to, &tmp__, 3);}
#define S24_3LE_NORMALIZE(from, to) {to = int24_to_int(from) / S24_SCALE;}
#define S24_3LE_ZERO(to)           {memset(to, 0, 3);}

#define S32_TYPE                   janus_int32_t
#define S32_BITS                   32
#define S32_SCALE                  JANUS_REAL_CONST(2147483648.0)
#define S32_QUANTIZE(from, to)     {to = (S32_TYPE)(from * S32_SCALE);}
#define S32_NORMALIZE(from, to)    {to = (from / S32_SCALE);}
#define S32_ZERO(to)               {to = 0;}

#define FLOAT_TYPE                 float
#define FLOAT_BITS                 0
#define FLOAT_SCALE                JANUS_REAL_CONST(1.0)
#define FLOAT_QUANTIZE(from, to)   {to = (FLOAT_TYPE)(from);}
#define FLOAT_NORMALIZE(from, to)  {to = (janus_real_t)(from);}
#define FLOAT_ZERO(to)             {to = 0;}

#define DOUBLE_TYPE                double
#define DOUBLE_BITS                0
#define DOUBLE_SCALE               JANUS_REAL_CONST(1.0)
#define DOUBLE_QUANTIZE(from, to)  {to = (DOUBLE_TYPE)(from);}
#define DOUBLE_NORMALIZE(from, to) {to = (janus_real_t)(from);}
#define DOUBLE_ZERO(to)            {to = 0;}

#define QUANTIZE_ILEAVE(type, in, in_count, out, chan_count, chan)      \
  {                                                                     \
    unsigned i = 0;                                                     \
    unsigned j = 0;                                                     \
    type ## _TYPE* qi_out__ = (type ## _TYPE*)out;                      \
                                                                        \
    for (i = 0; i < in_count; ++i)                                      \
    {                                                                   \
      for (j = 0; j < chan_count; ++j)                                  \
      {                                                                 \
        if (j == chan)                                                  \
        {                                                               \
          type ## _QUANTIZE(in[i], *qi_out__);                          \
        }                                                               \
        else                                                            \
        {                                                               \
          type ## _ZERO(*qi_out__);                                     \
        }                                                               \
                                                                        \
        ++qi_out__;                                                     \
      }                                                                 \
    }                                                                   \
  }

#define NORMALIZE_DLEAVE(type, in, in_count, out, chan_count, chan)     \
  {                                                                     \
    unsigned i = 0;                                                     \
    unsigned count__ = in_count * chan_count;                           \
    janus_real_t* nd_out__ = out;                                       \
    type ## _TYPE* nd_in__ = (type ## _TYPE*)in + chan;                 \
                                                                        \
    for (i = 0; i < count__; i += chan_count)                           \
    {                                                                   \
      type ## _NORMALIZE(*nd_in__, *nd_out__);                          \
      ++nd_out__;                                                       \
      nd_in__ += chan_count;                                            \
    }                                                                   \
  }

#define QUANTIZE_ILEAVE_COMPLEX(type, in, in_count, out)        \
  {                                                             \
    unsigned i = 0;                                             \
    type ## _TYPE* qic_out__ = (type ## _TYPE*)out;             \
                                                                \
    for (i = 0; i < in_count; ++i)                              \
    {                                                           \
      type ## _QUANTIZE(in[i][0], *qic_out__);                  \
      ++qic_out__;                                              \
      type ## _QUANTIZE(in[i][1], *qic_out__);                  \
      ++qic_out__;                                              \
    }                                                           \
  }

#define NORMALIZE_DLEAVE_COMPLEX(type, in, in_count, out)       \
  {                                                             \
    unsigned i = 0;                                             \
    unsigned count__ = in_count * 2;                            \
    janus_complex_t* out__ = (janus_complex_t*)out;             \
    type ## _TYPE* nd_in__ = (type ## _TYPE*)in;                \
                                                                \
    for (i = 0; i < count__; i += 2)                            \
    {                                                           \
      janus_real_t tr, ti;                                      \
      type ## _NORMALIZE(*nd_in__++, tr);                       \
      type ## _NORMALIZE(*nd_in__++, ti);                       \
      janus_complex_new(tr, ti, *out__);                        \
      ++out__;                                                  \
    }                                                           \
  }

// Define converters.
#define FORMAT(type)                                                    \
  static void                                                           \
  quantize_ ## type(const janus_real_t* samples, unsigned sample_count, \
                    void* frames,                                       \
                    unsigned channel_count, unsigned channel)           \
  {                                                                     \
    QUANTIZE_ILEAVE(type, samples, sample_count, frames,                \
                    channel_count, channel);                            \
  }                                                                     \
                                                                        \
  static void                                                           \
  normalize_ ## type(const void* frames,                                \
                     unsigned frame_count, janus_real_t* samples,       \
                     unsigned channel_count, unsigned channel)          \
  {                                                                     \
    NORMALIZE_DLEAVE(type, frames, frame_count, samples,                \
                     channel_count, channel);                           \
  }                                                                     \
                                                                        \
  static void                                                           \
  quantize_complex_## type(janus_complex_t* samples,                    \
                           unsigned sample_count,                       \
                           void* frames)                                \
  {                                                                     \
    QUANTIZE_ILEAVE_COMPLEX(type, samples, sample_count, frames);       \
  }                                                                     \
                                                                        \
  static void                                                           \
  normalize_complex_## type(const void* frames,                         \
                            unsigned frame_count,                       \
                            janus_complex_t* samples)                   \
  {                                                                     \
    NORMALIZE_DLEAVE_COMPLEX(type, frames, frame_count, samples);       \
  }

#include "format.def"

#define STR_VALUE(arg) #arg
#define QUOTE(x) STR_VALUE(x)

static char* janus_format_names[] =
{
  "UNKNOWN",
#define FORMAT(type) QUOTE(type),
#include "format.def"
};

const char*
janus_stream_format_get_name(janus_stream_format_t format)
{
  return janus_format_names[format];
}

janus_stream_format_t
janus_stream_format_parse(const char* format)
{
  size_t str_len = strlen(format);
  char* str = (char*)calloc(str_len + 1, 1);
  size_t i;
  janus_stream_format_t rv;

  // Convert format to upper case.
  for (i = 0; i < str_len; ++i)
    str[i] = toupper(format[i]);

#define FORMAT(a)                                                       \
  if (strcmp(str, #a) == 0) { rv = JANUS_STREAM_FORMAT_ ## a; } else
#include "format.def"
  {
  rv = JANUS_STREAM_FORMAT_UNKNOWN;
  }

  free(str);
  return rv;
}

unsigned
janus_stream_format_get_sample_size(janus_stream_format_t format)
{
  switch (format)
  {
#define FORMAT(a)                               \
    case JANUS_STREAM_FORMAT_ ## a:             \
      return sizeof(a ## _TYPE);
#include "format.def"
    default:
      return 0;
  }
}

unsigned
janus_stream_format_get_sample_size_bits(janus_stream_format_t format)
{
  switch (format)
  {
#define FORMAT(a)                               \
    case JANUS_STREAM_FORMAT_ ## a:             \
      return a ## _BITS;
#include "format.def"
    default:
      return 0;
  }
}

janus_stream_format_quantizer_t
janus_stream_format_get_quantizer(janus_stream_format_t format)
{
  switch (format)
  {
#define FORMAT(a)                               \
    case JANUS_STREAM_FORMAT_ ## a:             \
      return &quantize_ ## a;
#include "format.def"
    default:
      return NULL;
  }
}

janus_stream_format_normalizer_t
janus_stream_format_get_normalizer(janus_stream_format_t format)
{
  switch (format)
  {
#define FORMAT(a)                               \
    case JANUS_STREAM_FORMAT_ ## a:             \
      return &normalize_ ## a;
#include "format.def"
    default:
      return NULL;
  }
}

janus_stream_format_quantizer_complex_t
janus_stream_format_get_quantizer_complex(janus_stream_format_t format)
{
  switch (format)
  {
#define FORMAT(a)                               \
    case JANUS_STREAM_FORMAT_ ## a:             \
      return &quantize_complex_ ## a;
#include "format.def"
    default:
      return NULL;
  }
}

janus_stream_format_normalizer_complex_t
janus_stream_format_get_normalizer_complex(janus_stream_format_t format)
{
  switch (format)
  {
#define FORMAT(a)                               \
    case JANUS_STREAM_FORMAT_ ## a:             \
      return &normalize_complex_ ## a;
#include "format.def"
    default:
      return NULL;
  }
}
