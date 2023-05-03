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

#ifndef JANUS_STREAM_OSTREAM_H_INCLUDED_
#define JANUS_STREAM_OSTREAM_H_INCLUDED_

// JANUS headers.
#include <janus/config.h>
#include <janus/types.h>
#include <janus/duc.h>
#include <janus/utils/fifo.h>
#include <janus/stream/format.h>
#include <janus/export.h>

//! @ingroup STREAMS
//! @{
//! @defgroup OUTPUT Output
//! @{

#define JANUS_OSTREAM_PDATA                                             \
  struct private_data* pdata = (struct private_data*)ostream->pdata

typedef struct janus_ostream* janus_ostream_t;

struct janus_ostream
{
  //! Sampling frequency (Hz).
  unsigned fs;
  //! Sampling period (s).
  janus_real_t ts;
  //! Number of output stream channels.
  unsigned channel_count;
  //! Signal output channel.
  unsigned channel;
  //! Sample format.
  janus_stream_format_t format;
  //! Format quantizer.
  janus_stream_format_quantizer_t quantize;
  //! Format quantizer (complex).
  janus_stream_format_quantizer_complex_t quantize_complex;

  //! Sample size.
  unsigned sample_size;
  //! Frame size (in bytes).
  unsigned frame_size;

  //! Driver name.
  char name[36];
  //! Driver args.
  char args[256];

  //! Pointer to driver's open function.
  int (*open)(janus_ostream_t);
  //! Pointer to driver's close function.
  int (*close)(janus_ostream_t);
  //! Pointer to driver's reset function.
  int (*reset)(janus_ostream_t);
  //! Pointer to driver's free function.
  void (*free)(janus_ostream_t);
  //! Pointer to driver's status function.
  int (*status)(janus_ostream_t);
  //! Pointer to driver's read function.
  int (*read)(const janus_ostream_t, void*, unsigned);
  //! Pointer to driver's write function.
  int (*write)(janus_ostream_t, void*, unsigned);
  //! Pointer to driver's error translating function.
  const char* (*get_error)(janus_ostream_t);
  //! Pointer to driver's error translating function.
  const char* (*get_error_op)(janus_ostream_t);

  //! Private data used by stream implementation.
  void* pdata;
  //! Error message buffer.
  char bfr_error[128];

  //! Output buffer.
  void* out;
  //! Output buffer size.
  unsigned out_size;
  //! Output buffer capacity.
  unsigned out_capacity;
  //! Output buffer insert offset.
  unsigned out_offset;

  //! Upconverter.
  janus_duc_t duc;
  //! Upconverter output buffer.
  janus_real_t* duc_out;

  //! Amplitude factor.
  janus_real_t amp_factor;
  //! Force number of output samples to be a multiple.
  janus_uint8_t multiple;
};

//! Retrieve the list of drivers.
//! @return list of drivers.
JANUS_EXPORT const char**
janus_ostream_get_drivers(void);

//! Create output stream object.
//! @param driver driver's name.
//! @param args driver's arguments.
//! @return output stream object.
JANUS_EXPORT janus_ostream_t
janus_ostream_new(const char* driver, const char* args);

//! Free output stream.
//! @param ostream output stream.
JANUS_EXPORT void
janus_ostream_free(janus_ostream_t ostream);

//! Set the number of output stream channels.
//! @param ostream output stream object.
//! @param count number of output stream channels.
JANUS_EXPORT void
janus_ostream_set_channel_count(janus_ostream_t ostream, unsigned count);

//! Retrieve the number of output stream channels.
//! @param ostream input stream object.
//! @return number of output stream channels.
JANUS_EXPORT unsigned
janus_ostream_get_channel_count(janus_ostream_t ostream);

//! Set the signal output channel.
//! @param ostream output stream object.
//! @param channel signal output channel.
JANUS_EXPORT void
janus_ostream_set_channel(janus_ostream_t ostream, unsigned channel);

//! Retrieve the selected output channel.
//! @param ostream output stream object.
//! @return selected output channel.
JANUS_EXPORT unsigned
janus_ostream_get_channel(janus_ostream_t ostream);

//! Set sampling frequency.
//! @param ostream output stream object.
//! @param fs sampling frequency (Hz).
JANUS_EXPORT void
janus_ostream_set_fs(janus_ostream_t ostream, unsigned fs);

//! Retrieve sampling frequency.
//! @param ostream output stream object.
//! @return sampling frequency (Hz).
JANUS_EXPORT unsigned
janus_ostream_get_fs(janus_ostream_t ostream);

//! Set output stream format.
//! @param ostream output stream object.
//! @param format format.
JANUS_EXPORT void
janus_ostream_set_format(janus_ostream_t ostream, janus_stream_format_t format);

//! Retrieve output stream format.
//! @param ostream output stream object.
//! @return output stream format.
JANUS_EXPORT janus_stream_format_t
janus_ostream_get_format(janus_ostream_t ostream);

//! Retrieve frame size.
//! @param ostream output stream object.
//! @return frame size in bytes.
JANUS_EXPORT unsigned
janus_ostream_get_frame_size(janus_ostream_t ostream);

//! Retrieve sample size.
//! @param ostream output stream object.
//! @return sample size in bytes.
JANUS_EXPORT unsigned
janus_ostream_get_sample_size(janus_ostream_t ostream);

//! Set up converter.
//! @param ostream output stream object.
//! @param duc digital up converter object.
JANUS_EXPORT void
janus_ostream_set_duc(janus_ostream_t ostream, janus_duc_t duc);

//! Set amplitude factor.
//! @param factor amplitude factor.
JANUS_EXPORT void
janus_ostream_set_amplitude_factor(janus_ostream_t ostream, janus_real_t factor);

//! Set multiple.
//! @param multiple multiple.
JANUS_EXPORT void
janus_ostream_set_multiple(janus_ostream_t ostream, janus_uint8_t multiple);

//! Set capacity of internal buffer (in frames).
//! @param ostream output stream object.
//! @param size size of the output buffer.
JANUS_EXPORT void
janus_ostream_set_buffer_capacity(janus_ostream_t ostream, unsigned size);

//! Retrieve capacity of internal buffer (in frames).
//! @param ostream output stream object.
//! @return size of the output buffer.
JANUS_EXPORT unsigned
janus_ostream_get_buffer_capacity(janus_ostream_t ostream);

//! Open stream.
//! @param ostream output stream object.
JANUS_EXPORT int
janus_ostream_open(janus_ostream_t ostream);

//! Close stream.
//! @param ostream output stream object.
JANUS_EXPORT int
janus_ostream_close(janus_ostream_t ostream);

//! Reset stream.
//! @param ostream output stream object.
JANUS_EXPORT int
janus_ostream_reset(janus_ostream_t ostream);

//! Status query.
//! @param ostream output stream status.
JANUS_EXPORT int
janus_ostream_status(janus_ostream_t ostream);

//! Write samples to stream.
//! @param ostream output stream object.
//! @param samples array of samples.
//! @param sample_count number of samples.
//! @return 0 if success, otherwise error code.
JANUS_EXPORT int
janus_ostream_write(janus_ostream_t ostream, janus_complex_t* samples, unsigned sample_count);

//! Force write of all buffered data.
//! @param ostream output stream object.
//! @return 0 if success, otherwise error code.
JANUS_EXPORT int
janus_ostream_flush(janus_ostream_t ostream);

//! Retrieve the last error.
//! @param ostream output stream object.
//! @return last error.
JANUS_EXPORT const char*
janus_ostream_get_error(janus_ostream_t ostream);

//! Print stream parameters to standard output.
//! @param ostream output stream object.
JANUS_EXPORT void
janus_ostream_dump(janus_ostream_t ostream);

//! @}
//! @}

#endif
