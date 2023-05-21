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
// Author: Luigi Elia D'Amaro, Roberto Petroccia                          *
//*************************************************************************

#ifndef JANUS_CODEC_CODECS_H_INCLUDED_
#define JANUS_CODEC_CODECS_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>
#include <janus/defaults.h>
#include <janus/codec/fields.h>
#include <janus/codec/plugin.h>

struct janus_codecs
{
  janus_uint8_t verbose;
};

typedef struct janus_codecs* janus_codecs_t;

JANUS_EXPORT void
janus_codecs_reset(janus_codecs_t codecs);

JANUS_EXPORT void
janus_codecs_set_verbose(janus_codecs_t codecs, janus_uint8_t verbose);

JANUS_EXPORT int
janus_codecs_decode_app_data(janus_codecs_t codecs, janus_uint8_t class_id, janus_uint8_t app_type, janus_uint64_t app_data, janus_uint8_t app_data_size,
                             unsigned* cargo_size, janus_app_fields_t app_fields);

JANUS_EXPORT int 
janus_codecs_encode_app_data(janus_codecs_t codecs, janus_uint8_t class_id, janus_uint8_t app_type, unsigned desired_cargo_size, janus_app_fields_t app_fields, janus_uint8_t app_data_size,
                             unsigned* cargo_size, janus_uint64_t* app_data);

JANUS_EXPORT int
janus_codecs_decode_cargo(janus_codecs_t codecs, janus_uint8_t class_id, janus_uint8_t app_type, janus_uint8_t* cargo, unsigned cargo_size, janus_app_fields_t* app_fields);

JANUS_EXPORT int
janus_codecs_encode_cargo(janus_codecs_t codecs, janus_uint8_t class_id, janus_uint8_t app_type, janus_app_fields_t app_fields, janus_uint8_t** cargo, unsigned* cargo_size);

#endif
