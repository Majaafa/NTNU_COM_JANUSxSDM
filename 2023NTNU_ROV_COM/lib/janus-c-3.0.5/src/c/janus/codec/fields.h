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

#ifndef JANUS_APP_FIELDS_H_INCLUDED_
#define JANUS_APP_FIELDS_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>

struct janus_app_field
{
  char* name;
  char* value;
};

typedef struct janus_app_field* janus_app_field_t;

struct janus_app_fields
{
  janus_uint8_t field_count;
  janus_app_field_t fields;
};

typedef struct janus_app_fields* janus_app_fields_t;

JANUS_EXPORT janus_app_fields_t
janus_app_fields_new(void);

JANUS_EXPORT void
janus_app_fields_free(janus_app_fields_t app_fields);

JANUS_EXPORT void
janus_app_fields_reset(janus_app_fields_t app_fields);

JANUS_EXPORT void
janus_app_fields_add_field(janus_app_fields_t app_fields, const char* name, const char* value);

JANUS_EXPORT void
janus_app_fields_add_blob(janus_app_fields_t app_fields, const char* name, const janus_uint8_t* value, const unsigned size);

JANUS_EXPORT void
janus_app_fields_add_fields(janus_app_fields_t app_fields, const char* fields);

#endif
