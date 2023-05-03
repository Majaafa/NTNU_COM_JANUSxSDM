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

// ISO C headers.
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

// Local headers.
#include "string.h"

char*
cli_string_dup(const char* str)
{
  size_t len = strlen(str);
  char* dup = (char*)malloc(len + 1);
  strcpy(dup, str);
  return dup;
}

char*
cli_string_trim(char* str)
{
  char* dup = NULL;
  char* r = NULL;

  // Left trim.
  char* l = str;
  while (*l != '\0')
  {
    if (!isspace(*l))
      break;
    ++l;
  }

  dup = cli_string_dup(l);

  // Right trim.
  r = dup + strlen(dup) - 1;
  for (; isspace(*r); --r)
    *r = 0;

  return dup;
}
