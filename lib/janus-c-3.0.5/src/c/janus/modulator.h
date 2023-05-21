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

#ifndef JANUS_MODULATOR_H_INCLUDED_
#define JANUS_MODULATOR_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/pset.h>
#include <janus/export.h>
#include <janus/complex.h>
#include <janus/stream/ostream.h>

//! @ingroup MODEM
//! @{

typedef struct janus_modulator* janus_modulator_t;

JANUS_EXPORT janus_modulator_t
janus_modulator_new(janus_pset_t pset, unsigned fs);

JANUS_EXPORT void
janus_modulator_free(janus_modulator_t mod);

JANUS_EXPORT unsigned
janus_modulator_execute(janus_modulator_t mod, janus_uint8_t chip, janus_complex_t** out);

JANUS_EXPORT void
janus_modulator_reset(janus_modulator_t mod);

JANUS_EXPORT unsigned
janus_modulator_get_max_samples(janus_modulator_t mod);

//! @}

#endif
