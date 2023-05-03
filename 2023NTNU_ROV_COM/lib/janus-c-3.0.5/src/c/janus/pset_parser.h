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

#ifndef JANUS_PSET_PARSER_H_INCLUDED_
#define JANUS_PSET_PARSER_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>
#include <janus/pset.h>

//! @ingroup PSET
//! @{

typedef struct janus_pset_parser* janus_pset_parser_t;

JANUS_EXPORT janus_pset_parser_t
janus_pset_parser_new(const char* file);

JANUS_EXPORT void
janus_pset_parser_free(janus_pset_parser_t pset);

JANUS_EXPORT janus_pset_t
janus_pset_parser_get(janus_pset_parser_t pset);

//! @}

#endif
