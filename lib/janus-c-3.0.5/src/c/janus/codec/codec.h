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
// Author: Luigi Elia D'Amaro                                             *
//*************************************************************************

#ifndef JANUS_CODEC_CODEC_H_INCLUDED_
#define JANUS_CODEC_CODEC_H_INCLUDED_

// JANUS headers.
#include <janus/export.h>
#include <janus/codec/plugin.h>

#ifdef JANUS_PLUGINS_DEBUG
   #define DEBUG_PRINT(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__ );
#else
   #define DEBUG_PRINT(fmt, ...) ;/* commented #x */
#endif

JANUS_PLUGIN_EXPORT int
app_data_decode(janus_uint64_t app_data, janus_uint8_t app_data_size, unsigned* cargo_size, janus_app_fields_t app_fields);

JANUS_PLUGIN_EXPORT int
app_data_encode(unsigned desired_cargo_size, janus_app_fields_t app_fields, janus_uint8_t app_data_size, unsigned* cargo_size, janus_uint64_t* app_data);

JANUS_PLUGIN_EXPORT int
cargo_decode(janus_uint8_t* cargo, unsigned cargo_size, janus_app_fields_t* app_fields);

JANUS_PLUGIN_EXPORT int
cargo_encode(janus_app_fields_t app_fields, janus_uint8_t** cargo, unsigned* cargo_size);

#endif
