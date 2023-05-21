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
// Authors: Ricardo Martins, Luigi Elia D'Amaro, Roberto Petroccia,       *
//          Giovanni Zappa                                                *
//*************************************************************************

// ISO C headers.
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Local headers.
#include "options.h"
#include "string.h"

// JANUS headers.
#include <janus/error.h>
#include <janus/dump.h>
#include <janus/utils/memory.h>

static int
cli_options_parse(int argc, char** argv, struct cli_option* opts)
{
  int i = 0;
  int j = 0;
  int last_opt = 0;

  if (argc == 0)
    return 0;

  for (i = 0; i < argc; ++i)
  {
    if (strcmp(argv[i], "--help") == 0)
      return 0;

    if (argv[i][0] == '-' && argv[i][1] == '-')
    {
      for (j = 0; j < OPTIONS_MAX; ++j)
      {
        if (strlen(argv[i]) > 3)
        {
          if (strcmp(argv[i] + 2, opts[j].option) == 0)
          {
            if (!opts[j].is_present)
            {
              last_opt = i;
              opts[j].is_present = 1;
              if (i == (argc - 1))
                return 0;

              last_opt = i + 1;
              strncpy(opts[j].arg, argv[i + 1], ARG_LEN - 1);
              opts[j].arg[ARG_LEN - 1] = '\0';
            }
            ++i;
            break;
          }
        }
      }
    }
  }

  return last_opt;
}

static int
cli_options_parse_file(const char* file, char*** argv)
{
  int i = 0;
  char opt[512*4];
  char arg[512*4];
  char* str = NULL;
  int argc = 0;
  int argc_max = 10;
  char line[1024*4];
  char** pargv = NULL;
  int rv;

  FILE* fd = fopen(file, "r");
  if (fd == NULL)
    return 0;

  *argv = (char**)malloc(argc_max * sizeof(char*));
  pargv = *argv;

  while (!feof(fd))
  {
    if (fscanf(fd, "%2049[^\n]\n", line) != 1)
    {
      break;
    }

    str = cli_string_trim(line);

    if ((str[0] != '-') || (str[1] != '-'))
    {
      free(str);
      continue;
    }

    rv = sscanf(str, "%511[^ ] %2047[^\n]\n", opt, arg);
    free(str);
    if (rv != 2)
    {
      fprintf(stderr, "invalid format\n");
      goto cleanup;
    }

    pargv[argc++] = cli_string_trim(opt);
    if (argc == argc_max)
    {
      argc_max += 10;
      *argv = (char**)realloc(*argv, sizeof(char*) * argc_max);
      pargv = *argv;
    }

    pargv[argc++] = cli_string_trim(arg);
    if (argc == argc_max)
    {
      argc_max += 10;
      *argv = (char**)realloc(*argv, sizeof(char*) * argc_max);
      pargv = *argv;
    }
  }

  fclose(fd);
  pargv[argc] = NULL;
  return argc;

 cleanup:
  for (i = 0; i < argc; ++i)
    free(pargv[i]);
  free(pargv);

  fclose(fd);
  return 0 ;
}

cli_options_t
cli_options_new(int argc, char** argv)
{
  cli_options_t cli_options = JANUS_UTILS_MEMORY_NEW_ZERO(struct cli_options, 1);

  cli_options->file_argv = 0;

  #define OPTION(a, b, c, d, e) \
    strncpy(cli_options->opts[a].option, b, OPTION_LEN - 1); \
    cli_options->opts[a].option[OPTION_LEN - 1] = '\0';
  #include "cli/options.def"

  #define OPTION(a, b, c, d, e) \
    strncpy(cli_options->opts[a].desc, c, DESC_LEN - 1); \
    cli_options->opts[a].desc[DESC_LEN - 1] = '\0';
  #include "cli/options.def"

  #define OPTION(a, b, c, d, e) \
    cli_options->opts[a].is_present = d;
  #include "cli/options.def"

  #define OPTION(a, b, c, d, e) \
    strncpy(cli_options->opts[a].arg, e, ARG_LEN - 1); \
    cli_options->opts[a].arg[ARG_LEN - 1] = '\0';
  #include "cli/options.def"

  if (argc > 1)
    cli_options->last_opt = cli_options_parse(argc - 1, argv + 1, cli_options->opts);

  // Parse options from file.
  if (cli_options->opts[CONFIG_FILE].is_present)
  {
    int file_argc = cli_options_parse_file(cli_options->opts[CONFIG_FILE].arg, &(cli_options->file_argv));
    cli_options_parse(file_argc, cli_options->file_argv, cli_options->opts);
  }

  return cli_options;
}

void
cli_options_free(cli_options_t cli_options)
{
  if (cli_options->file_argv)
  {
    char** ptr = cli_options->file_argv;
    while (*ptr)
    {
      free(*ptr);
      ++ptr;
    }
    free(cli_options->file_argv);
  }

  JANUS_UTILS_MEMORY_FREE(cli_options);
}

int
cli_options_is_valid(cli_options_t cli_options)
{
  return (cli_options->last_opt > 0);
}

void
cli_options_usage(cli_options_t cli_options)
{
  unsigned i;
  for (i = 0; i < OPTIONS_MAX; ++i)
  {
    fprintf(stderr, "  --%-25s %s\n", cli_options->opts[i].option, cli_options->opts[i].desc);
  }
}

void
cli_options_dump(cli_options_t cli_options)
{
  #define JANUS_OPTION_DUMP(o) JANUS_DUMP("Options", o.desc, "%s", o.arg)
  #define OPTION(a, b, c, d, e) JANUS_OPTION_DUMP(cli_options->opts[a]);
  #include "cli/options.def"
}

void
cli_options_get_params(cli_options_t cli_options, janus_parameters_t params)
{
  // Parameters.
  params->verbose = cli_options->opts[VERBOSE].is_present ? atoi(cli_options->opts[VERBOSE].arg) : 0;
  params->pset_id = atoi(cli_options->opts[PSET_ID].arg);
  params->pset_file = cli_options->opts[PSET_FILE].arg;
  params->pset_center_freq = atoi(cli_options->opts[PSET_CENTER_FREQ].arg);
  params->pset_bandwidth = atoi(cli_options->opts[PSET_BANDWIDTH].arg);
  params->chip_len_exp = atoi(cli_options->opts[CHIP_LEN_EXP].arg);
  if (cli_options->opts[SEQUENCE_32_CHIPS].is_present)
  {
    janus_uint32_t sequence_32_chips;
    sscanf(cli_options->opts[SEQUENCE_32_CHIPS].arg, "%X", &sequence_32_chips);
    params->sequence_32_chips = sequence_32_chips;
  }

  // Stream parameters.
  params->stream_driver = cli_options->opts[STREAM_DRIVER].arg;
  params->stream_driver_args = cli_options->opts[STREAM_DRIVER_ARGS].arg;
  params->stream_fs = atoi(cli_options->opts[STREAM_FS].arg);
  params->stream_format = cli_options->opts[STREAM_FORMAT].arg;
  params->stream_channel = atoi(cli_options->opts[STREAM_CHANNEL].arg);
  params->stream_channel_count = atoi(cli_options->opts[STREAM_CHANNELS].arg);
  params->stream_passband = atoi(cli_options->opts[STREAM_PASSBAND].arg);

#ifdef CLI_TX
  // Tx parameters.
  params->pad = atoi(cli_options->opts[PAD].arg);
  params->wut = atoi(cli_options->opts[WUT].arg);
  params->stream_amp = atof(cli_options->opts[STREAM_AMP].arg);
  params->stream_mul = atoi(cli_options->opts[STREAM_MUL].arg);
#endif

#ifdef CLI_RX
  // Rx parameters.
  params->doppler_correction = atoi(cli_options->opts[DOPPLER_CORRECTION].arg);
  params->doppler_max_speed = atof(cli_options->opts[DOPPLER_MAX_SPEED].arg);
  params->compute_channel_spectrogram = atoi(cli_options->opts[CHANNEL_SPECTROGRAM].arg);
  params->detection_threshold = atof(cli_options->opts[DETECTION_THRESHOLD].arg);
  params->colored_bit_prob    = atoi(cli_options->opts[COLORED_BIT_PROB].arg);
  params->cbp_high2medium     = atof(cli_options->opts[CBP_HIGH2MEDIUM].arg);
  params->cbp_medium2low      = atof(cli_options->opts[CBP_MEDIUM2LOW].arg);
  params->rx_once             = atoi(cli_options->opts[RX_ONCE].arg);
  params->skip_detection      = atoi(cli_options->opts[SKIP_DETECTION].arg);
  params->detected_offset     = atoi(cli_options->opts[DETECTED_OFFSET].arg);
  params->detected_doppler    = atof(cli_options->opts[DETECTED_DOPPLER].arg);
#endif
}

int
cli_options_get_packet(cli_options_t cli_options, janus_packet_t packet)
{
#ifdef CLI_TX
  const char* cargo = 0;

  janus_packet_set_mobility(packet, atoi(cli_options->opts[PACKET_MOBILITY].arg));
  janus_packet_set_tx_rx(packet, atoi(cli_options->opts[PACKET_TX_RX].arg));
  janus_packet_set_forward(packet, atoi(cli_options->opts[PACKET_FORWARD].arg));
  janus_packet_set_class_id(packet, atoi(cli_options->opts[PACKET_CLASS_ID].arg));
  janus_packet_set_app_type(packet, atoi(cli_options->opts[PACKET_APP_TYPE].arg));

  if (cli_options->opts[PACKET_RESERV_TIME].is_present)
  {
    double dvalue = atof(cli_options->opts[PACKET_RESERV_TIME].arg);
    if (dvalue > 0.0)
    {
      double evalue1, evalue2;
      enum janus_packet_interval_lookup_result interval_error;
      interval_error = janus_packet_set_tx_reservation_time(packet, dvalue, &evalue1, &evalue2);
      if (interval_error == JANUS_PACKET_BETWEEN_TWO_VALUES)
      {
        interval_error = janus_packet_set_tx_reservation_time(packet, evalue2, &evalue1, &evalue2);
      }
      if (interval_error != JANUS_PACKET_EXACT_TIME && interval_error != JANUS_PACKET_APPROXIMATED_TIME)
      {
        fprintf(stderr, "ERROR: unknown reservation time: %f\n", dvalue);
        return 1;
      }
    }
  }
  else if (cli_options->opts[PACKET_REPEAT_INT].is_present)
  {
    double dvalue = atof(cli_options->opts[PACKET_REPEAT_INT].arg);
    if (dvalue > 0.0)
    {
      double evalue1, evalue2;
      enum janus_packet_interval_lookup_result interval_error;
      interval_error = janus_packet_set_tx_repeat_interval(packet, dvalue, &evalue1, &evalue2);
      if (interval_error == JANUS_PACKET_BETWEEN_TWO_VALUES)
      {
        interval_error = janus_packet_set_tx_repeat_interval(packet, evalue1, &evalue1, &evalue2);
      }
      if (interval_error != JANUS_PACKET_EXACT_TIME && interval_error != JANUS_PACKET_APPROXIMATED_TIME)
      {
        fprintf(stderr, "ERROR: unknown repeat interval: %f\n", dvalue);
        return 1;
      }
    }
  }

  janus_packet_set_validity(packet, 1);
  if (cli_options->opts[PACKET_APP_FIELDS].is_present)
  {
    int res;
    janus_app_fields_t app_fields = janus_app_fields_new();
    janus_app_fields_add_fields(app_fields, cli_options->opts[PACKET_APP_FIELDS].arg);
    janus_packet_set_application_data_fields(packet, app_fields);
    
    res = janus_packet_encode_cargo(packet);
    if (res != JANUS_ERROR_NONE)
    {
      janus_packet_set_cargo_error(packet, res);
      fprintf(stderr, "ERROR: failed to encode cargo from data fields %d\n", res);
          return 1;
    }
    if (janus_packet_get_desired_cargo_size(packet) > 0)
      janus_packet_set_validity(packet, 2);

    janus_packet_encode_application_data(packet);
    
    if (res != JANUS_ERROR_NONE)
    {
      janus_packet_set_validity(packet, 0);
      fprintf(stderr, "ERROR: failed to encode application data fields %d\n", res);
      return 1;
    }
    
    janus_app_fields_free(app_fields);
  }


  // Set application data.
  if (cli_options->opts[PACKET_APP_DATA].is_present)
  {
    janus_uint64_t app_data = 0;
    sscanf(cli_options->opts[PACKET_APP_DATA].arg, "%llX", &app_data);
    janus_packet_set_application_data(packet, app_data);
  }
  
  // Set cargo if provided.
  if (cli_options->opts[PACKET_CARGO].is_present)
  {
    int cargo_error;
    cargo = cli_options->opts[PACKET_CARGO].arg;
    unsigned desired_cargo_size = (unsigned)strlen(cargo);
    cargo_error = janus_packet_set_cargo(packet, (janus_uint8_t*)cargo, desired_cargo_size);
    if (cargo_error == JANUS_ERROR_CARGO_SIZE)
    {
      fprintf(stderr, "ERROR: cargo size : %d exceeding maximum value\n", desired_cargo_size);
      return 1;
    }
    if (janus_packet_get_desired_cargo_size(packet))
    {
      if (! cli_options->opts[PACKET_APP_DATA].is_present)
      {
        janus_packet_encode_application_data(packet);
      }
      else
      {
        janus_packet_set_cargo_size(packet, desired_cargo_size);
      }
      janus_packet_set_validity(packet, 2);
    }
  }  
  
#endif

  return JANUS_ERROR_NONE;
}
