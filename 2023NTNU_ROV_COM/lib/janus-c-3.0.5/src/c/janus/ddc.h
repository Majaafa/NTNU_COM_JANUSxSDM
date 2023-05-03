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

#ifndef JANUS_DDC_H_INCLUDED_
#define JANUS_DDC_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>
#include <janus/complex.h>
#include <janus/utils/fifo.h>

//! @ingroup HETERODYNE
//! @{

//! Digital Down Converter (DDC).
typedef struct janus_ddc* janus_ddc_t;

//! Create DDC object.
//! @param input_fs input sampling frequency (Hz).
//! @param cfreq carrier frequency (Hz).
//! @param bwidth bandwidth (Hz).
//! @return DDC object.
JANUS_EXPORT janus_ddc_t
janus_ddc_new(unsigned input_fs, unsigned cfreq, unsigned bwidth);

//! Free DDC object.
//! @param ddc DDC object.
JANUS_EXPORT void
janus_ddc_free(janus_ddc_t ddc);

//! Retrieve the ratio between the input and output sampling
//! frequencies.
//! @param ddc DDC object.
//! @return ratio between input and output sampling frequencies.
JANUS_EXPORT unsigned
janus_ddc_get_ratio(janus_ddc_t ddc);

//! Retrieve the output sampling frequency.
//! @param ddc DDC object.
//! @return output sampling frequency (Hz).
JANUS_EXPORT unsigned
janus_ddc_get_output_fs(janus_ddc_t ddc);

//! Retrieve the number of samples produced by a single call
//! to janus_ddc_execute.
//! @param ddc DDC object.
//! @return number of produced samples.
JANUS_EXPORT unsigned
janus_ddc_get_output_sample_count(janus_ddc_t ddc);

//! Retrieve the number of samples consumed by a single call to
//! janus_ddc_execute.
//! @param ddc DDC object.
//! @return number of consumed samples.
JANUS_EXPORT unsigned
janus_ddc_get_input_sample_count(janus_ddc_t ddc);

//! Downconvert samples.
//! @param ddc DDC object.
//! @param pband passband samples, size must be the same as
//! janus_ddc_get_input_sample_count.
//! @param bband baseband samples, size must be the same as
//! janus_ddc_get_output_sample_count.
//! @return number of samples produced.
JANUS_EXPORT unsigned
janus_ddc_execute(janus_ddc_t ddc, janus_real_t* pband, janus_complex_t* bband);

//! @}

#endif
