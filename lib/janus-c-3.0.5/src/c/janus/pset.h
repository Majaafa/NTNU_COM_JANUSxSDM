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

#ifndef JANUS_PSET_H_INCLUDED_
#define JANUS_PSET_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/primitive.h>
#include <janus/export.h>

//! @ingroup PSET
//! @{

//! Parameter set object.
typedef struct janus_pset* janus_pset_t;

//! Structure representing a parameter set and associated values.
struct janus_pset
{
  //! Parameter set ID.
  unsigned id;
  //! Center frequency (Hz).
  unsigned cfreq;
  //! Available bandwidth (Hz).
  unsigned abwidth;
  //! Used bandwidth (Hz).
  unsigned ubwidth;
  //! Chip duration.
  janus_real_t chip_dur;
  //! Chip frequency.
  unsigned chip_frq;
  //! Chip length multiplier.
  janus_real_t chip_len_mul;
  //! Number of frequency blocks.
  unsigned frq_block_count;
  //! Primitive, computed when calling janus_pset_compute.
  struct janus_primitive primitive;
  //! 32 chips sequence.
  janus_uint32_t c32_sequence;
  //! Name of parameter set.
  char name[36];
};

JANUS_EXPORT janus_pset_t
janus_pset_new(void);

JANUS_EXPORT void
janus_pset_free(janus_pset_t pset);

JANUS_EXPORT int
janus_pset_load(janus_pset_t pset, const char* file, unsigned id);

JANUS_EXPORT void
janus_pset_set_bwidth(janus_pset_t pset, unsigned value);

JANUS_EXPORT unsigned
janus_pset_get_bwidth(janus_pset_t pset);

JANUS_EXPORT void
janus_pset_set_cfreq(janus_pset_t pset, unsigned value);

JANUS_EXPORT unsigned
janus_pset_get_cfreq(janus_pset_t pset);

JANUS_EXPORT void
janus_pset_set_id(janus_pset_t pset, unsigned value);

JANUS_EXPORT unsigned
janus_pset_get_id(janus_pset_t pset);

JANUS_EXPORT void
janus_pset_set_chip_len_exp(janus_pset_t pset, janus_uint16_t chip_len_exp);

JANUS_EXPORT void
janus_pset_set_32_chip_sequence(janus_pset_t pset, janus_uint32_t sequence);

JANUS_EXPORT void
janus_pset_set_name(janus_pset_t pset, const char* name);

JANUS_EXPORT int
janus_pset_is_valid(janus_pset_t pset);

JANUS_EXPORT void
janus_pset_dump(const janus_pset_t pset);

//! @}

#endif
