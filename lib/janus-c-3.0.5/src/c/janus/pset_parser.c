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
#include <stdio.h>

// JANUS headers.
#include <janus/pset_parser.h>
#include <janus/utils/memory.h>

struct janus_pset_parser
{
  FILE* fd;
};

janus_pset_parser_t
janus_pset_parser_new(const char* file)
{
  janus_pset_parser_t parser;

  FILE* fd = fopen(file, "r");
  if (fd == NULL)
    return NULL;

  parser = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_pset_parser, 1);
  parser->fd = fd;
  return parser;
}

void
janus_pset_parser_free(janus_pset_parser_t parser)
{
  fclose(parser->fd);
  JANUS_UTILS_MEMORY_FREE(parser);
}

janus_pset_t
janus_pset_parser_get(janus_pset_parser_t parser)
{
  while (!feof(parser->fd))
  {
    unsigned id;
    unsigned cfreq;
    unsigned abwidth;
    char name[36];
    int matches = fscanf(parser->fd, "%u , %u , %u , %35[^\n]\n",
                         &id, &cfreq, &abwidth, name);

    if (matches == 4)
    {
      janus_pset_t pset = janus_pset_new();
      janus_pset_set_id(pset, id);
      janus_pset_set_cfreq(pset, cfreq);
      janus_pset_set_bwidth(pset, abwidth);
      janus_pset_set_name(pset, name);
      return pset;
    }

    matches = fscanf(parser->fd, "%*[^\n]\n");
  }

  return NULL;
}
