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
#include <stdio.h>
#include <string.h>

// JANUS headers.
#include <janus/codec/fields.h>
#include <janus/utils/memory.h>

static char*
string_dup(const char* str)
{
  size_t len = strlen(str);
  char* dup = (char*)malloc(len + 1);
  strcpy(dup, str);
  return dup;
}

janus_app_fields_t
janus_app_fields_new(void)
{
  janus_app_fields_t app_fields = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_app_fields, 1);

  return app_fields;
}

void
janus_app_fields_free(janus_app_fields_t app_fields)
{
  unsigned i;
  for (i = 0; i < app_fields->field_count; ++i)
  {
    free(app_fields->fields[i].name);
    free(app_fields->fields[i].value);
  }
  JANUS_UTILS_MEMORY_FREE(app_fields->fields);

  JANUS_UTILS_MEMORY_FREE(app_fields);
}

void
janus_app_fields_reset(janus_app_fields_t app_fields)
{
  while (app_fields->field_count > 0)
  {
    free(app_fields->fields[app_fields->field_count - 1].name);
    free(app_fields->fields[app_fields->field_count - 1].value);
    app_fields->field_count--;
  }
}

void
janus_app_fields_add_field(janus_app_fields_t app_fields, const char* name, const char* value)
{
  janus_uint8_t i = app_fields->field_count++;
  app_fields->fields = JANUS_UTILS_MEMORY_REALLOC(app_fields->fields, struct janus_app_field, app_fields->field_count);
  
  app_fields->fields[i].name = string_dup(name);
  app_fields->fields[i].value = string_dup(value);
}

void
janus_app_fields_add_blob(janus_app_fields_t app_fields, const char* name, const janus_uint8_t* value, const unsigned size)
{
  janus_uint8_t i = app_fields->field_count++;
  app_fields->fields = JANUS_UTILS_MEMORY_REALLOC(app_fields->fields, struct janus_app_field, app_fields->field_count);
  
  app_fields->fields[i].name = string_dup(name);
  app_fields->fields[i].value = malloc((size + 1) *sizeof(janus_uint8_t) );
  memset(app_fields->fields[i].value, 0x0, size+1);
  memcpy(app_fields->fields[i].value, value, size);
}

void
janus_app_fields_add_fields(janus_app_fields_t app_fields, const char* fields)
{
  char name[256];
  char value[1024];
  const char name_delim[] = "=";
  const char value_delim[] = ",";
  char* token;
  char* string = string_dup(fields);

  token = strtok(string, name_delim);
  while (token != NULL)
  {
    strcpy(name, token);

    token = strtok(NULL, value_delim);
    if (token == NULL)
      break;

    strcpy(value, token);

    janus_app_fields_add_field(app_fields, name, value);

    token = strtok(NULL, name_delim);
  }
  
  free(string);
}
