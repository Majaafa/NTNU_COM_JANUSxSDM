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

// ISO C headers.
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// JANUS headers.
#include <janus/codec/codecs.h>
#include <janus/utils/memory.h>

static janus_plugin_t plugins[JANUS_CLASS_COUNT][JANUS_APP_COUNT];
static janus_app_data_decode_function_t app_data_decode_func[JANUS_CLASS_COUNT][JANUS_APP_COUNT];
static janus_app_data_encode_function_t app_data_encode_func[JANUS_CLASS_COUNT][JANUS_APP_COUNT];
static janus_cargo_decode_function_t cargo_decode_func[JANUS_CLASS_COUNT][JANUS_APP_COUNT];
static janus_cargo_encode_function_t cargo_encode_func[JANUS_CLASS_COUNT][JANUS_APP_COUNT];

static janus_plugin_t
janus_codecs_get_plugin(janus_codecs_t codecs, janus_uint8_t class_id, janus_uint8_t app_type)
{
  if (plugins[class_id][app_type] == NULL)
  {
    char plugin_name[256];
    janus_plugin_t plugin;
#if (defined(_WIN32) || defined(__WIN32__))
    sprintf(plugin_name, "plugin_%03u_%02u.dll", class_id, app_type);
#else
    sprintf(plugin_name, "libplugin_%03u_%02u.so", class_id, app_type);
#endif
    plugin = janus_plugin_open(plugin_name);

    if (plugin)
      plugins[class_id][app_type] = plugin;
    else if (codecs->verbose)
      fprintf(stderr, "-> Plugin %s not opened (%s)\n", plugin_name, janus_plugin_error(plugin));
  }

  return plugins[class_id][app_type];
}

void
janus_codecs_reset(janus_codecs_t codecs)
{
  for (int i = 0; i != JANUS_CLASS_COUNT; ++i)
  {
    for (int j = 0; j != JANUS_APP_COUNT; ++j)
    {
      if (plugins[i][i] != NULL)
      {
        janus_plugin_close(plugins[i][j]);
      }
    }
  }
  
  memset(plugins, 0, JANUS_CLASS_COUNT * JANUS_APP_COUNT * sizeof(void*));
  memset(app_data_decode_func, 0, JANUS_CLASS_COUNT * JANUS_APP_COUNT * sizeof(void*));
  memset(app_data_encode_func, 0, JANUS_CLASS_COUNT * JANUS_APP_COUNT * sizeof(void*));
  memset(cargo_decode_func, 0, JANUS_CLASS_COUNT * JANUS_APP_COUNT * sizeof(void*));
  memset(cargo_encode_func, 0, JANUS_CLASS_COUNT * JANUS_APP_COUNT * sizeof(void*));
}

void
janus_codecs_set_verbose(janus_codecs_t codecs, janus_uint8_t verbose)
{
  codecs->verbose = verbose;
}

int
janus_codecs_decode_app_data(janus_codecs_t codecs, janus_uint8_t class_id, janus_uint8_t app_type, janus_uint64_t app_data, janus_uint8_t app_data_size,
                             unsigned* cargo_size, janus_app_fields_t app_fields)
{
  int rv = 0;

  janus_app_data_decode_function_t decode = app_data_decode_func[class_id][app_type];
  if (decode == NULL)
  {
    janus_plugin_t plugin = janus_codecs_get_plugin(codecs, class_id, app_type);
    decode = janus_plugin_get_app_data_decode_function(plugin);
    app_data_decode_func[class_id][app_type] = decode;
  }
  if (decode != NULL)
  {
    rv = (*decode)(app_data, app_data_size, cargo_size, app_fields);
  }

  return rv;
}

int
janus_codecs_encode_app_data(janus_codecs_t codecs, janus_uint8_t class_id, janus_uint8_t app_type, unsigned desired_cargo_size, janus_app_fields_t app_fields, janus_uint8_t app_data_size,
                             unsigned* cargo_size, janus_uint64_t* app_data)
{
  int rv = 0;

  janus_app_data_encode_function_t encode = app_data_encode_func[class_id][app_type];
  if (encode == NULL)
  {
    janus_plugin_t plugin = janus_codecs_get_plugin(codecs, class_id, app_type);
    encode = janus_plugin_get_app_data_encode_function(plugin);
    app_data_encode_func[class_id][app_type] = encode;
  }
  if (encode != NULL)
  {
    rv = (*encode)(desired_cargo_size, app_fields, app_data_size, cargo_size, app_data);
  }

  return rv;
}

int
janus_codecs_decode_cargo(janus_codecs_t codecs, janus_uint8_t class_id, janus_uint8_t app_type, janus_uint8_t* cargo, unsigned cargo_size,
                            janus_app_fields_t* app_fields)
{
  int rv = 0;

  janus_cargo_decode_function_t decode = cargo_decode_func[class_id][app_type];
  if (decode == NULL)
  {
    janus_plugin_t plugin = janus_codecs_get_plugin(codecs, class_id, app_type);
    decode = janus_plugin_get_cargo_decode_function(plugin);
    cargo_decode_func[class_id][app_type] = decode;
  }
  if (decode != NULL)
  {
    rv = (*decode)(cargo, cargo_size, app_fields);
  }

  return rv;
}

int
janus_codecs_encode_cargo(janus_codecs_t codecs, janus_uint8_t class_id, janus_uint8_t app_type, janus_app_fields_t app_fields,
                            janus_uint8_t** cargo, unsigned* cargo_size)
{
  int rv = 0;

  janus_cargo_encode_function_t encode = cargo_encode_func[class_id][app_type];
  if (encode == NULL)
  {
    janus_plugin_t plugin = janus_codecs_get_plugin(codecs, class_id, app_type);
    encode = janus_plugin_get_cargo_encode_function(plugin);
    cargo_encode_func[class_id][app_type] = encode;
  }
  if (encode != NULL)
  {
    rv = (*encode)(app_fields, cargo, cargo_size);
  }

  return rv;
}

