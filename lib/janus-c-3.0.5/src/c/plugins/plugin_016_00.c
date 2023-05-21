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

// ISO C headers.
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// JANUS headers.
#include <janus/defaults.h>
#include <janus/codec/codec.h>
#include <janus/error.h>

#define STATION_ID_LABEL "Station_Identifier"
#define PSET_ID_LABEL "Parameter Set Identifier"
#define PAYLOAD_SIZE_LABEL "Payload Size"
#define PAYLOAD_LABEL "Payload"

#define BMASK(var, bit)  ((var >> (64 - bit)) & 0x00000001U)
#define LMASK(var, bits) (var & (0xFFFFFFFFFFFFFFFFU >> (64 - bits)))
#define HMASK(var, bits) (var & (0xFFFFFFFFFFFFFFFFU << (64 - bits)))

static inline unsigned
cargo_lookup_index(unsigned index)
{
  if (index == 0)
    return 0;
  else if (index < 4)
    return 1 << (index - 1);
  else
    return 8 * (index - 3);
}

static inline unsigned
cargo_lookup_size(unsigned dsize, unsigned* esize)
{
  unsigned index = 0;

  if (dsize < 3)
  {
    index = dsize;
    *esize = dsize;
  }
  else if (dsize < 5)
  {
    index = 3;
    *esize = 4;
  }
  else if (dsize <= 480)
  {
    *esize = ((dsize - 1) & 0xfff8) + 8;
    index = 4 + ((*esize - 8) / 8);
  }
  else
  {
    *esize = 0;
  }

  return index;
}

static inline void
app_data_decode_station_id(janus_uint64_t app_data, janus_app_fields_t app_fields)
{
  char name[] = STATION_ID_LABEL;
  char value[4];

  janus_uint8_t station_id = (janus_uint8_t)((app_data >> 18) & (0xFFU));
  sprintf(value, "%u", station_id);

  janus_app_fields_add_field(app_fields, name, value);
}

static inline void
app_data_decode_pset_id(janus_uint64_t app_data, janus_app_fields_t app_fields)
{
  char name[] = PSET_ID_LABEL;
  char value[5];

  janus_uint16_t pset_id = (janus_uint16_t)((app_data >> 6) & (0xFFFU));
  sprintf(value, "%u", pset_id);

  janus_app_fields_add_field(app_fields, name, value);
}

static inline unsigned
app_data_decode_cargo_size(janus_uint64_t app_data)
{
  unsigned cargo_size_index = (unsigned)(app_data & (0x3FU));
  return cargo_lookup_index(cargo_size_index);
}

static inline void
app_fields_encode_station_id(janus_uint64_t* app_data, janus_app_field_t app_data_field)
{
  janus_uint64_t station_id = atoi(app_data_field->value);
  *app_data = HMASK(*app_data, 38) | (station_id << 18) | LMASK(*app_data, 18);
}

static inline void
app_fields_encode_pset_id(janus_uint64_t* app_data, janus_app_field_t app_data_field)
{
  janus_uint64_t pset_id = atoi(app_data_field->value);
  *app_data = HMASK(*app_data, 46) | (pset_id << 6) | LMASK(*app_data, 6);
}

static inline unsigned 
app_fields_encode_cargo_size(janus_uint64_t* app_data, unsigned desired_cargo_size)
{
  unsigned cargo_size;
  janus_uint64_t cargo_size_index = cargo_lookup_size(desired_cargo_size, &cargo_size);
  *app_data = HMASK(*app_data, 58) | cargo_size_index;
  return cargo_size;
}

JANUS_PLUGIN_EXPORT int
app_data_decode(janus_uint64_t app_data, janus_uint8_t app_data_size, unsigned* cargo_size, janus_app_fields_t app_fields)
{
  // Station_Identifier (8 bits).
  app_data_decode_station_id(app_data, app_fields);

  // Parameter Set Identifier (12 bits).
  app_data_decode_pset_id(app_data, app_fields);

  // Cargo Size (6 bits).
  *cargo_size = app_data_decode_cargo_size(app_data);

  return 0;
}

JANUS_PLUGIN_EXPORT int
app_data_encode(unsigned desired_cargo_size, janus_app_fields_t app_fields, janus_uint8_t app_data_size, unsigned* cargo_size, janus_uint64_t* app_data)
{
  *app_data = 0;

  // Check cargo size validity.
  if (desired_cargo_size > 480)
    return JANUS_ERROR_CARGO_SIZE;

  // Cargo Size (6 bits).
  *cargo_size = app_fields_encode_cargo_size(app_data, desired_cargo_size);

  if (app_fields != 0)
  {
    int i;
    for (i = 0; i < app_fields->field_count; ++i)
    {
      if (strcmp(app_fields->fields[i].name, STATION_ID_LABEL) == 0)
      {
        // Station_Identifier (8 bits).
        app_fields_encode_station_id(app_data, app_fields->fields + i);
      }
      else if (strcmp(app_fields->fields[i].name, PSET_ID_LABEL) == 0)
      {
        // Parameter Set Identifier (12 bits).
        app_fields_encode_pset_id(app_data, app_fields->fields + i);
      }
      else
      {
        //return -1;
      }
    }
  }
  
  return 0;
}

JANUS_PLUGIN_EXPORT int
cargo_decode(janus_uint8_t* cargo, unsigned cargo_size, janus_app_fields_t* app_fields)
{
  int rv = 0;

  if (*app_fields == 0)
  {
    *app_fields = janus_app_fields_new();
  }

  char size_string[4];
  sprintf(size_string, "%3u", cargo_size);

  janus_app_fields_add_field(*app_fields, PAYLOAD_SIZE_LABEL, size_string);

  janus_app_fields_add_blob(*app_fields, PAYLOAD_LABEL, cargo, cargo_size);

  return rv;
}

JANUS_PLUGIN_EXPORT int
cargo_encode(janus_app_fields_t app_fields, janus_uint8_t** cargo, unsigned* cargo_size)
{
  int rv = JANUS_ERROR_FIELDS;
  int cargo_size_found = 0;

  unsigned i;
  for (i = 0; i != app_fields->field_count; ++i)
  {
    if (strcmp(app_fields->fields[i].name, PAYLOAD_SIZE_LABEL) == 0)
    {
      *cargo_size = atoi(app_fields->fields[i].value);
      cargo_size_found = 1;
      break;
    }
  }

  for (i = 0; i != app_fields->field_count; ++i)
  {
    if (strcmp(app_fields->fields[i].name, PAYLOAD_LABEL) == 0)
    {
      if (! cargo_size_found)
      {
        *cargo_size = strlen(app_fields->fields[i].value);
      }
      if (*cargo_size > JANUS_MAX_PKT_CARGO_SIZE)
      {
        return JANUS_ERROR_CARGO_SIZE;
      }
      
      *cargo = JANUS_UTILS_MEMORY_REALLOC(*cargo, janus_uint8_t, *cargo_size);
      memcpy(*cargo, app_fields->fields[i].value, *cargo_size * sizeof(janus_uint8_t));
      
      rv = 0;
      
      break;
    }
  }
  if (*cargo_size == 0)
    return 0;
  else
    return rv;
}
