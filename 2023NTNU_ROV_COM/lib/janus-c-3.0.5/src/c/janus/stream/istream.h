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
// Authors: Ricardo Martins, Luigi Elia D'Amaro                           *
//*************************************************************************

#ifndef JANUS_STREAM_ISTREAM_H_INCLUDED_
#define JANUS_STREAM_ISTREAM_H_INCLUDED_

// JANUS headers.
#include <janus/config.h>
#include <janus/types.h>
#include <janus/ddc.h>
#include <janus/utils/fifo.h>
#include <janus/stream/format.h>
#include <janus/export.h>

//! @ingroup STREAMS
//! @{
//! @defgroup INPUT Input
//! @{

#define JANUS_ISTREAM_PDATA                                          \
  struct private_data* pdata = (struct private_data*)istream->pdata

typedef struct janus_istream* janus_istream_t;

struct janus_istream
{
  //! Sampling frequency (Hz).
  unsigned fs;
  //! Sampling period (s).
  janus_real_t ts;
  //! Number of input stream channels.
  unsigned channel_count;
  //! Selected input channel (all others are ignored).
  unsigned channel;
  //! Sample format.
  janus_stream_format_t format;
  //! Sample normalizer.
  janus_stream_format_normalizer_t normalize;
  //! Sample normalizer (complex).
  janus_stream_format_normalizer_complex_t normalize_complex;

  //! Sample size.
  unsigned sample_size;
  //! Frame size (in bytes).
  unsigned frame_size;

  //! Driver name.
  char name[36];
  //! Driver args.
  char args[256];

  //! Pointer to driver's open function.
  int (*open)(janus_istream_t);
  //! Pointer to driver's close function.
  int (*close)(janus_istream_t);
  //! Pointer to driver's free function.
  void (*free)(janus_istream_t);
  //! Pointer to driver's read function.
  int (*read)(const janus_istream_t, void*, unsigned);
  //! Pointer to driver's write function.
  int (*write)(janus_istream_t, void*, unsigned);
  //! Pointer to driver's error translating function.
  const char* (*get_error)(janus_istream_t);
  //! Pointer to driver's error translating function.
  const char* (*get_error_op)(janus_istream_t);

  //! Private data used by stream implementation.
  void* pdata;
  //! Error message buffer.
  char bfr_error[128];

  //! Output buffer.
  janus_complex_t* out;
  //! Output buffer size (in samples).
  unsigned out_size;

  //! Downconverter.
  janus_ddc_t ddc;
  //! Downconverter input buffer.
  janus_real_t* ddc_in;
  //! Downconverter input buffer size (in samples).
  unsigned ddc_in_size;
  
  //! Driver input buffer.
  void* drv_in;
  //! Driver input buffer size (in frames).
  unsigned drv_in_size;
};

//! Retrieve the list of drivers.
//! @return list of drivers.
JANUS_EXPORT const char**
janus_istream_get_drivers(void);

//! Create input stream object.
//! @param driver driver's name.
//! @param args driver's arguments.
//! @return input stream object.
JANUS_EXPORT janus_istream_t
janus_istream_new(const char* driver, const char* args);

//! Free input stream.
//! @param istream input stream.
JANUS_EXPORT void
janus_istream_free(janus_istream_t istream);

//! Set the number of input stream channels.
//! @param istream input stream object.
//! @param count number of input stream channels.
JANUS_EXPORT void
janus_istream_set_channel_count(janus_istream_t istream, unsigned count);

//! Retrieve the number of input stream channels.
//! @param istream input stream object.
//! @return number of input stream channels.
JANUS_EXPORT unsigned
janus_istream_get_channel_count(janus_istream_t istream);

//! Set selected input channel (all others are ignored).
//! @param istream input stream object.
//! @param active selected input channel.
JANUS_EXPORT void
janus_istream_set_channel(janus_istream_t istream, unsigned channel);

//! Retrieve the selected input channel.
//! @param istream input stream object.
//! @return selected input channel.
JANUS_EXPORT unsigned
janus_istream_get_channel(janus_istream_t istream);

//! Set sampling frequency.
//! @param istream input stream object.
//! @param fs sampling frequency (Hz).
JANUS_EXPORT void
janus_istream_set_fs(janus_istream_t istream, unsigned fs);

//! Retrieve sampling frequency.
//! @param istream input stream object.
//! @return sampling frequency (Hz).
JANUS_EXPORT unsigned
janus_istream_get_fs(janus_istream_t istream);

//! Set input stream format.
//! @param istream input stream object.
//! @param format format.
JANUS_EXPORT void
janus_istream_set_format(janus_istream_t istream, janus_stream_format_t format);

//! Retrieve input stream format.
//! @param istream input stream object.
//! @return input stream format.
JANUS_EXPORT janus_stream_format_t
janus_istream_get_format(janus_istream_t istream);

//! Retrieve frame size.
//! @param istream input stream object.
//! @return frame size in bytes.
JANUS_EXPORT unsigned
janus_istream_get_frame_size(janus_istream_t istream);

//! Retrieve sample size.
//! @param istream input stream object.
//! @return sample size in bytes.
JANUS_EXPORT unsigned
janus_istream_get_sample_size(janus_istream_t istream);

//! Set down converter.
//! @param istream input stream object.
//! @param ddc digital down converter object.
JANUS_EXPORT void
janus_istream_set_ddc(janus_istream_t istream, janus_ddc_t ddc);

//! Retrieve the sampling frequency of the output signal.
//! @param istream input stream object.
//! @return sampling frequency (Hz).
JANUS_EXPORT unsigned
janus_istream_get_output_fs(janus_istream_t istream);

//! Open stream.
//! @param istream input stream object.
JANUS_EXPORT int
janus_istream_open(janus_istream_t istream);

//! Close stream.
//! @param istream input stream object.
JANUS_EXPORT int
janus_istream_close(janus_istream_t istream);

//! Read samples from stream.
//! @param istream input stream object.
//! @param samples array of samples.
//! @return number of read samples.
JANUS_EXPORT int
janus_istream_read(janus_istream_t istream, janus_complex_t** samples);

//! Retrieve the last error.
//! @param istream input stream object.
//! @return last error.
JANUS_EXPORT const char*
janus_istream_get_error(janus_istream_t istream);

//! Print stream parameters to standard output.
//! @param istream input stream object.
JANUS_EXPORT void
janus_istream_dump(janus_istream_t istream);

//! @}
//! @}

#endif
