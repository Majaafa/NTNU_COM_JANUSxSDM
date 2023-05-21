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
// Authors: Ricardo Martins, Luigi Elia D'Amaro, Roberto Petroccia        *
//*************************************************************************

#ifndef CLI_OPTIONS_H_INCLUDED_
#define CLI_OPTIONS_H_INCLUDED_

// JANUS headers.
#include <janus/defaults.h>
#include <janus/parameters.h>
#include <janus/packet.h>

#define OPTION_LEN   30
#define DESC_LEN     50
#define ARG_LEN      (2048)

enum
{
#define OPTION(a, b, c, d, e) a,
#include "cli/options.def"
  OPTIONS_MAX
};

struct cli_option
{
  char option[OPTION_LEN];
  char desc[DESC_LEN];
  int is_present;
  char arg[ARG_LEN];
};

struct cli_options
{
  struct cli_option opts[OPTIONS_MAX];
  int last_opt;
  char** file_argv;
};

typedef struct cli_options* cli_options_t;

cli_options_t
cli_options_new(int argc, char** argv);

void
cli_options_free(cli_options_t cli_options);

int
cli_options_is_valid(cli_options_t cli_options);

void
cli_options_usage(cli_options_t cli_options);

void
cli_options_dump(cli_options_t cli_options);

void
cli_options_get_params(cli_options_t cli_options, janus_parameters_t params);

int
cli_options_get_packet(cli_options_t cli_options, janus_packet_t packet);

#endif
