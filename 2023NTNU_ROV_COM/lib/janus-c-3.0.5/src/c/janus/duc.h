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

#ifndef JANUS_DUC_H_INCLUDED_
#define JANUS_DUC_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/complex.h>
#include <janus/export.h>

//! @ingroup HETERODYNE
//! @{

//! Digital Up Converter (DUC).
typedef struct janus_duc* janus_duc_t;

//! Create DUC object.
//! @param fs sampling frequency (Hz).
//! @param cfreq carrier frequency (Hz).
//! @return DUC object.
JANUS_EXPORT janus_duc_t
janus_duc_new(unsigned fs, unsigned cfreq);

//! Free DUC object.
//! @param duc DUC object.
JANUS_EXPORT void
janus_duc_free(janus_duc_t duc);

JANUS_EXPORT void
janus_duc_execute(janus_duc_t duc, janus_complex_t* samples_in, unsigned samples_in_count, janus_real_t* samples_out, janus_real_t amp);

JANUS_EXPORT void
janus_duc_reset(janus_duc_t duc);

//! @}

#endif
