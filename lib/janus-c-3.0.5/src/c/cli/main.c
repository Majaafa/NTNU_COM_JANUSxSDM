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
// Authors: Ricardo Martins, Luigi Elia D'Amaro, Giovanni Zappa           *
//*************************************************************************

// ISO C headers.
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>

// JANUS headers.
#include <janus/janus.h>
#include <janus/defaults.h>

// Local headers.
#include "options.h"

#ifdef CLI_TX
#  define PROG_NAME "janus-tx"
#endif

#ifdef CLI_RX
#  define PROG_NAME "janus-rx"
#endif

static int done = 0;

static void
signal_handler_callback(int signum)
{
  done = 1;
}

static void
signal_handler_init()
{
#ifdef CLI_RX
  // Register signal and signal handler.
  signal(SIGABRT, signal_handler_callback);
  signal(SIGFPE, signal_handler_callback);
  signal(SIGILL, signal_handler_callback);
  signal(SIGINT, signal_handler_callback);
  signal(SIGSEGV, signal_handler_callback);
  signal(SIGTERM, signal_handler_callback);
  #if !(defined(_WIN32) || defined(__WIN32__))
  signal(SIGHUP, signal_handler_callback);
  signal(SIGPIPE, signal_handler_callback);
  signal(SIGQUIT, signal_handler_callback);
  #endif
#endif
}

int
main(int argc, char** argv)
{
  cli_options_t cli_options = 0;
  janus_parameters_t params = 0;

  cli_options = cli_options_new(argc, argv);
  if (!cli_options_is_valid(cli_options))
  {
    fprintf(stderr, "Usage: " PROG_NAME " [OPTIONS]\n");
    fprintf(stderr, "JANUS is a simple, robust signaling method for underwater communications.\n\n");
    cli_options_usage(cli_options);
    fprintf(stderr, "\nCopyright (C) 2008-2018 STO Centre for Maritime Research and Experimentation (CMRE)\n");
    cli_options_free(cli_options);
    return 1;
  }

  signal_handler_init();

  // Parameters.
  params = janus_parameters_new();
  cli_options_get_params(cli_options, params);
  if (params->verbose > 1)
    cli_options_dump(cli_options);

#ifdef CLI_TX
  {
    janus_simple_tx_t simple_tx = 0;
    janus_packet_t packet = 0;
    janus_tx_state_t state = 0;

    // Initialize simple tx.
    simple_tx = janus_simple_tx_new(params);
    if (!simple_tx)
    {
      fprintf(stderr, "ERROR: failed to initialize transmitter.\n");
      janus_parameters_free(params);
      cli_options_free(cli_options);
      return 1;
    }

    // Initialize packet.
    packet = janus_packet_new(params->verbose);
    if (cli_options_get_packet(cli_options, packet) != JANUS_ERROR_NONE)
    {
      fprintf(stderr, "ERROR: failed to initialize packet.\n");
      janus_packet_free(packet);
      janus_simple_tx_free(simple_tx);
      janus_parameters_free(params);
      cli_options_free(cli_options);
      return 1;
    }

    // Initialize state.
    state = janus_tx_state_new((params->verbose > 1));

    // Transmit.
    janus_simple_tx_execute(simple_tx, packet, state);

    // Dump.
    if (params->verbose > 0)
    {
      janus_tx_state_dump(state);
      janus_packet_dump(packet);
    }

    // Cleanup.
    janus_tx_state_free(state);
    janus_packet_free(packet);
    janus_simple_tx_free(simple_tx);
  }
#endif

#ifdef CLI_RX
  {
    janus_simple_rx_t simple_rx = 0;
    janus_packet_t packet = 0;
    janus_rx_state_t state = 0;
    janus_carrier_sensing_t carrier_sensing = 0;
    unsigned queried_detection_time = 0;
    janus_real_t time;

    // Initialize rx.
    simple_rx = janus_simple_rx_new(params);
    if (!simple_rx)
    {
      fprintf(stderr, "ERROR: failed to initialize receiver.\n");
      janus_parameters_free(params);
      cli_options_free(cli_options);
      return 1;
    }

    // Initialize carrier sensing;
    carrier_sensing = janus_carrier_sensing_new(janus_simple_rx_get_rx(simple_rx));

    // Initialize packet.
    packet = janus_packet_new(params->verbose);

    // Initialize state.
    state = janus_rx_state_new(params);
    if (!state)
    {
      fprintf(stderr, "ERROR: failed to initialize receiver.\n");
      janus_parameters_free(params);
      cli_options_free(cli_options);
      return 1;
    }

    // Receive.
    while (!done)
    {
      int rv = janus_simple_rx_execute(simple_rx, packet, state);

      if (rv < 0)
      {
        if (rv == JANUS_ERROR_OVERRUN)
        {
          fprintf(stderr, "Error buffer-overrun\n");
        }
        break;
      }

      if (rv > 0)
      {
        // Dump.
        // Incorrect packet only if verbosity >= 3
        if (params->verbose >= 3  ||
            (params->verbose > 0 &&
             (janus_packet_get_validity(packet) &&
              janus_packet_get_cargo_error(packet) == 0)))
        {
          janus_rx_state_dump(state);
          janus_packet_dump(packet);
          janus_packet_reset(packet);
        }
        queried_detection_time = 0;
        janus_carrier_sensing_reset(carrier_sensing);
      }
      else
      {
        if (params->verbose > 0 && janus_simple_rx_has_detected(simple_rx) && !queried_detection_time)
        {
          fprintf(stderr, "-> Triggering detection (%.6f)\n", janus_simple_rx_get_first_detection_time(simple_rx));
          queried_detection_time = 1;
        }

        if (janus_carrier_sensing_execute(carrier_sensing, &time) > 0){}
          /*fprintf(stdout, "-> Busy channel t=%f w=%e b=%e\n", time,
            janus_carrier_sensing_window_power(carrier_sensing),
            janus_carrier_sensing_background_power(carrier_sensing));*/
      }
    }

    // Cleanup.
    janus_rx_state_free(state);
    janus_packet_free(packet);
    janus_carrier_sensing_free(carrier_sensing);
    janus_simple_rx_free(simple_rx);
  }
#endif

  janus_parameters_free(params);
  cli_options_free(cli_options);

  return 0;
}
