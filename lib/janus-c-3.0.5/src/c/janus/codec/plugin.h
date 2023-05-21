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

#ifndef JANUS_CODEC_PLUGIN_H_INCLUDED_
#define JANUS_CODEC_PLUGIN_H_INCLUDED_

// JANUS headers.
#include <janus/export.h>
#include <janus/codec/fields.h>
#include <janus/utils/memory.h>

#if (defined(_WIN32) || defined(__WIN32__))
#  include <windows.h>
#  define janus_plugin_t HMODULE
#else
#  include <dlfcn.h>
#  define janus_plugin_t void*
#endif
 
typedef int (*janus_app_data_decode_function_t) (janus_uint64_t app_data, janus_uint8_t app_data_size, unsigned* cargo_size, janus_app_fields_t app_fields);
typedef int (*janus_app_data_encode_function_t) (unsigned desired_cargo_size, janus_app_fields_t app_fields, janus_uint8_t app_data_size, unsigned* cargo_size, janus_uint64_t* app_data);
typedef int (*janus_cargo_decode_function_t) (janus_uint8_t* cargo, unsigned cargo_size, janus_app_fields_t* app_fields);
typedef int (*janus_cargo_encode_function_t) (janus_app_fields_t app_fields, janus_uint8_t** cargo, unsigned* cargo_size);

JANUS_EXPORT janus_plugin_t
janus_plugin_open(const char* plugin_name);

JANUS_EXPORT void
janus_plugin_close(janus_plugin_t plugin);

JANUS_EXPORT char* 
janus_plugin_error(janus_plugin_t plugin);

JANUS_EXPORT janus_app_data_decode_function_t
janus_plugin_get_app_data_decode_function(janus_plugin_t plugin);

JANUS_EXPORT janus_app_data_encode_function_t
janus_plugin_get_app_data_encode_function(janus_plugin_t plugin);

JANUS_EXPORT janus_cargo_decode_function_t
janus_plugin_get_cargo_decode_function(janus_plugin_t plugin);

JANUS_EXPORT janus_cargo_encode_function_t
janus_plugin_get_cargo_encode_function(janus_plugin_t plugin);

#endif
