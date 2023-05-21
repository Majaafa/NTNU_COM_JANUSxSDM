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
#include <string.h>

// JANUS headers.
#include <janus/defaults.h>
#include <janus/janus.h>
#include <janus/utils/memory.h>
#include <janus/dump.h>

// CLI headers.
#include <cli/options.h>

// Local headers.
#include <mex/mex_utils.h>

static void
janus_rx_mex_check_arguments(int nargout, int nargin, const mxArray* pargin[])
{
  unsigned i;

  if (nargout > 2)
  {
    mexErrMsgIdAndTxt("MATLAB:janus_rx_mex:maxlhs", "Too many output arguments.");
  }

  if (nargin < 2)
  {
    mexErrMsgIdAndTxt("MATLAB:janus_rx_mex:minrhs", "Not enough input arguments.");
  }

  i = 0;
  if (mex_utils_is_string(nargin, i, pargin))
  {
    mex_utils_check_string(nargin, i++, pargin);
    mex_utils_check_scalar(nargin, i++, pargin);
  }
  else
  {
    mex_utils_check_scalar(nargin, i++, pargin);
    mex_utils_check_scalar(nargin, i++, pargin);
  }
}

static void
janus_rx_mex_read_input_arguments(cli_options_t cli_options, int nargin, const mxArray* pargin[])
{
  int rv = 0;
  unsigned i, j, k;
  char field[ARG_LEN];
  char value[ARG_LEN];

  for (i = 0; i < OPTIONS_MAX; i++)
  {
    unsigned opt_len = (unsigned)strlen(cli_options->opts[i].option);
    for (j = 0; j < opt_len; j++)
      if (cli_options->opts[i].option[j] == '-')
        cli_options->opts[i].option[j] = '_';
  }
  
  i = 0;
  if (mex_utils_is_string(nargin, i, pargin))
  {
    mex_utils_get_string(nargin, i++, pargin, &(cli_options->opts[PSET_FILE]));
    mex_utils_get_scalar(nargin, i++, pargin, &(cli_options->opts[PSET_ID]));
  }
  else
  {
    mex_utils_get_scalar(nargin, i++, pargin, &(cli_options->opts[PSET_CENTER_FREQ]));
    mex_utils_get_scalar(nargin, i++, pargin, &(cli_options->opts[PSET_BANDWIDTH]));
  }

  for (; i < (unsigned)nargin; i++)
  {
    if (mex_utils_is_string(nargin, i, pargin))
    {
      rv = mxGetString(pargin[i], field, ARG_LEN);
      if (rv == 1)
      {
        printf("ERROR: failed to read input argument %d\n", i + 1);
        continue;
      }
      
      for (j = 0; j < OPTIONS_MAX; j++)
      {
        if (strcmp(cli_options->opts[j].option, field) == 0)
        {
          i++;
          if (i == nargin)
          {
            mexErrMsgIdAndTxt("MATLAB:janus_rx_mex:invarg", "Field and value input arguments must come in pairs.");
          }
          
          if (mex_utils_is_string(nargin, i, pargin))
          {
            rv = mxGetString(pargin[i], value, ARG_LEN);
            if (rv == 1)
            {
              printf("ERROR: failed to read input argument %d\n", i);
              continue;
            }
          }
          else if (mex_utils_is_scalar(nargin, i, pargin))
          {
            if (strcmp("packet_cargo", field) == 0)
            {
              size_t cargo_size = mxGetNumberOfElements(pargin[i]);
              double* data = (double*)mxGetData(pargin[i]);
              for (k = 0; k < cargo_size; k++)
              {
                value[k] = (char)(*(data + k));
              }
              value[k] = 0;
            }
            else {
              sprintf(value, "%f", (double)mxGetScalar(pargin[i]));
            }
          }
          else
          {
            mexErrMsgIdAndTxt("MATLAB:janus_rx_mex:invarg", "Unknown value type.");
            continue;
          }

          strncpy((char*)(cli_options->opts[j].arg), value, ARG_LEN - 1);
          cli_options->opts[j].arg[ARG_LEN - 1] = '\0';
          cli_options->opts[j].is_present = 1;
        }
      }
    }
    else
    {
      mexErrMsgIdAndTxt("MATLAB:janus_rx_mex:invarg", "Field names must be strings.");
    }
  }
}

static void
janus_rx_mex_write_output_arguments(int nargout, mxArray* pargout[], janus_simple_rx_t simple_rx, const janus_packet_t packet, janus_rx_state_t state)
{
  unsigned i;
  int nfields = 0;
  mxArray* field_value = 0;
  mxArray* app_data_field_value = 0;
  double* data;
  janus_uint8_t app_data_size;

  if (nargout > 0)
  {
    struct
    {
      const janus_uint8_t* bytes;
      janus_uint8_t version;
      janus_uint8_t mobility;
      janus_uint8_t schedule;
      janus_uint8_t tx_rx;
      janus_uint8_t forward;
      janus_uint8_t class_id;
      janus_uint8_t app_type;
      janus_uint8_t res_rep;
      double tx_interval;
      janus_uint32_t cargo_size;
      janus_app_fields_t app_fields;
      janus_uint64_t app_data;
      janus_uint8_t crc;
      janus_uint8_t crc_validity;
      const janus_uint8_t* cargo;
    } packet_fields;

    memset(&packet_fields, 0, sizeof(packet_fields));

    packet_fields.bytes = janus_packet_get_bytes(packet);
    packet_fields.version = janus_packet_get_version(packet);
    packet_fields.mobility = janus_packet_get_mobility(packet);
    packet_fields.schedule = janus_packet_get_schedule(packet);
    packet_fields.tx_rx = janus_packet_get_tx_rx(packet);
    packet_fields.forward = janus_packet_get_forward(packet);
    packet_fields.class_id = janus_packet_get_class_id(packet);
    packet_fields.app_type = janus_packet_get_app_type(packet);
    packet_fields.tx_interval = janus_packet_get_tx_interval(packet, &packet_fields.res_rep);
    packet_fields.app_fields = janus_app_fields_new();
    janus_packet_get_application_data_fields(packet, &packet_fields.cargo_size, packet_fields.app_fields);
    packet_fields.app_data = janus_packet_get_application_data(packet, &app_data_size);
    packet_fields.crc = janus_packet_get_crc(packet);
    packet_fields.crc_validity = janus_packet_get_crc_validity(packet);
    packet_fields.cargo = janus_packet_get_cargo(packet);

    nfields = 14 + (packet_fields.schedule ? 2 : 0);

    {
      const char** field_names = 0;
      i = 0;
      field_names = JANUS_UTILS_MEMORY_NEW_ZERO(const char*, nfields);
      field_names[i++] = "bytes";
      field_names[i++] = "version";
      field_names[i++] = "mobility";
      field_names[i++] = "schedule";
      field_names[i++] = "tx_rx";
      field_names[i++] = "forward";
      field_names[i++] = "class_id";
      field_names[i++] = "app_type";
      if (packet_fields.schedule)
      {
        field_names[i++] = "reserv_repeat";
        field_names[i++] = (packet_fields.res_rep == 0) ? "reserv_time" : "repeat_int";
      }
      field_names[i++] = "cargo_size";
      field_names[i++] = "app_fields";
      field_names[i++] = "app_data";
      field_names[i++] = "crc";
      field_names[i++] = "crc_validity";
      field_names[i++] = "cargo";

      pargout[0] = mxCreateStructMatrix(1, 1, nfields, field_names);
    }

    field_value = mxCreateNumericMatrix(1, JANUS_MIN_PKT_SIZE, mxDOUBLE_CLASS, mxREAL);
    data = mxGetPr(field_value);
    for (i = 0; i < JANUS_MIN_PKT_SIZE; ++i)
    {
      data[i] = (double)packet_fields.bytes[i];
    }
    mxSetField(pargout[0], 0, "bytes", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = packet_fields.version;
    mxSetField(pargout[0], 0, "version", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = packet_fields.mobility;
    mxSetField(pargout[0], 0, "mobility", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = packet_fields.schedule;
    mxSetField(pargout[0], 0, "schedule", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = packet_fields.tx_rx;
    mxSetField(pargout[0], 0, "tx_rx", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = packet_fields.forward;
    mxSetField(pargout[0], 0, "forward", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = packet_fields.class_id;
    mxSetField(pargout[0], 0, "class_id", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = packet_fields.app_type;
    mxSetField(pargout[0], 0, "app_type", field_value);

    if (packet_fields.schedule)
    {
      field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
      *mxGetPr(field_value) = packet_fields.res_rep;
      mxSetField(pargout[0], 0, "res_rep", field_value);

      field_value = mxCreateDoubleScalar(packet_fields.tx_interval);
      mxSetField(pargout[0], 0, (packet_fields.res_rep == 0) ? "reserv_time" : "repeat_int", field_value);
    }

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = packet_fields.cargo_size;
    mxSetField(pargout[0], 0, "cargo_size", field_value);

    {
      const char** app_fields_names = 0;
      i = 0;
      app_fields_names = JANUS_UTILS_MEMORY_NEW_ZERO(const char*, packet_fields.app_fields->field_count);
      for (i = 0; i < packet_fields.app_fields->field_count; ++i)
      {
        app_fields_names[i] = packet_fields.app_fields->fields[i].name;
      }

      field_value = mxCreateStructMatrix(1, 1, packet_fields.app_fields->field_count, app_fields_names);
      for (i = 0; i < packet_fields.app_fields->field_count; ++i)
      {
        app_data_field_value = mxCreateString(packet_fields.app_fields->fields[i].value);
        mxSetField(field_value, 0, packet_fields.app_fields->fields[i].name, app_data_field_value);
      }
      mxSetField(pargout[0], 0, "app_fields", field_value);
    }

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = (double)packet_fields.app_data;
    mxSetField(pargout[0], 0, "app_data", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = packet_fields.crc;
    mxSetField(pargout[0], 0, "crc", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = packet_fields.crc_validity;
    mxSetField(pargout[0], 0, "crc_validity", field_value);

    field_value = mxCreateNumericMatrix(1, packet_fields.cargo_size, mxDOUBLE_CLASS, mxREAL);
    data = mxGetPr(field_value);
    for (i = 0; i < packet_fields.cargo_size; ++i)
    {
      data[i] = (double)packet_fields.cargo[i];
    }
    mxSetField(pargout[0], 0, "cargo", field_value);
  }

  if (nargout > 1)
  {
    struct
    {
      unsigned prim_q;
      unsigned prim_a;
      unsigned nblock;
    } mex_state;

    memset(&mex_state, 0, sizeof(mex_state));

    mex_state.prim_q = janus_simple_rx_get_pset(simple_rx)->primitive.q;
    mex_state.prim_a = janus_simple_rx_get_pset(simple_rx)->primitive.alpha;
    mex_state.nblock = janus_simple_rx_get_pset(simple_rx)->frq_block_count;

    nfields = 13;

    {
      const char** field_names = 0;
      i = 0;
      field_names = JANUS_UTILS_MEMORY_NEW_ZERO(const char*, nfields);
      field_names[i++] = "pset_id";
      field_names[i++] = "pset_name";
      field_names[i++] = "cfreq";
      field_names[i++] = "bwidth";
      field_names[i++] = "chip_frq";
      field_names[i++] = "chip_dur";
      field_names[i++] = "prim_q";
      field_names[i++] = "prim_a";
      field_names[i++] = "nblock";
      field_names[i++] = "after";
      field_names[i++] = "gamma";
      field_names[i++] = "speed";
      field_names[i++] = "bit_prob";

      pargout[1] = mxCreateStructMatrix(1, 1, nfields, field_names);
    }

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = state->pset_id;
    mxSetField(pargout[1], 0, "pset_id", field_value);

    field_value = mxCreateString(state->pset_name);
    mxSetField(pargout[1], 0, "pset_name", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = state->cfreq;
    mxSetField(pargout[1], 0, "cfreq", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = state->bwidth;
    mxSetField(pargout[1], 0, "bwidth", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = state->chip_frq;
    mxSetField(pargout[1], 0, "chip_frq", field_value);

    field_value = mxCreateDoubleScalar(state->chip_dur);
    mxSetField(pargout[1], 0, "chip_dur", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = mex_state.prim_q;
    mxSetField(pargout[1], 0, "prim_q", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = mex_state.prim_a;
    mxSetField(pargout[1], 0, "prim_a", field_value);

    field_value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    *mxGetPr(field_value) = mex_state.nblock;
    mxSetField(pargout[1], 0, "nblock", field_value);

    field_value = mxCreateDoubleScalar(state->after);
    mxSetField(pargout[1], 0, "after", field_value);

    field_value = mxCreateDoubleScalar(state->gamma);
    mxSetField(pargout[1], 0, "gamma", field_value);

    field_value = mxCreateDoubleScalar(state->speed);
    mxSetField(pargout[1], 0, "speed", field_value);

    if (state->bit_prob_size > 0)
    {
      double* data = NULL;
      field_value = mxCreateNumericMatrix(1, state->bit_prob_size, mxDOUBLE_CLASS, mxREAL);
      data = mxGetPr(field_value);
      for (i = 0; i < (unsigned)state->bit_prob_size; ++i)
      {
        data[i] = *(state->bit_prob + i);
      }
      mxSetField(pargout[1], 0, "bit_prob", field_value);
    }
  }
}

static void
janus_rx_mex_free(cli_options_t cli_options, janus_parameters_t params, janus_simple_rx_t simple_rx, janus_packet_t packet, janus_rx_state_t state, janus_carrier_sensing_t carrier_sensing)
{
  if (carrier_sensing)
    janus_carrier_sensing_free(carrier_sensing);

  if (state)
    janus_rx_state_free(state);

  if (packet)
    janus_packet_free(packet);

  if (simple_rx)
    janus_simple_rx_free(simple_rx);

  if (params)
    janus_parameters_free(params);

  if (cli_options)
    cli_options_free(cli_options);
}

void
mexFunction(int nargout, mxArray* pargout[], int nargin, const mxArray* pargin[])
{
  cli_options_t cli_options = 0;
  janus_parameters_t params = 0;
  janus_simple_rx_t simple_rx = 0;
  janus_packet_t packet = 0;
  janus_rx_state_t state = 0;
  janus_carrier_sensing_t carrier_sensing = 0;
  unsigned queried_detection_time = 0;
  janus_real_t time;

  cli_options = cli_options_new(0, 0);

  // Check mex arguments.
  janus_rx_mex_check_arguments(nargout, nargin, pargin);

  // Read mex input arguments.
  janus_rx_mex_read_input_arguments(cli_options, nargin, pargin);

  // Parameters.
  params = janus_parameters_new();
  cli_options_get_params(cli_options, params);
  if (params->verbose > 1)
    cli_options_dump(cli_options);

  // Initialize simple rx.
  simple_rx = janus_simple_rx_new(params);

  // Initialize carrier sensing;
  carrier_sensing = janus_carrier_sensing_new(janus_simple_rx_get_rx(simple_rx));

  // Initialize packet.
  packet = janus_packet_new(params->verbose);

  // Initialize state.
  state = janus_rx_state_new((params->verbose > 1));

  // Receive.
  while (1)
  {
    int rv = janus_simple_rx_execute(simple_rx, packet, state);

    if (rv < 0)
      break;

    if (rv > 0)
    {
      if (params->verbose > 0)
        janus_rx_state_dump(state);
      janus_packet_dump(packet);
      break;
    }
    else
    {
      if (params->verbose > 0 && janus_simple_rx_has_detected(simple_rx) && !queried_detection_time)
      {
        fprintf(stderr, "-> triggering detection (%.6f)\n", janus_simple_rx_get_first_detection_time(simple_rx));
        queried_detection_time = 1;
      }

      if (janus_carrier_sensing_execute(carrier_sensing, &time) > 0)
        fprintf(stderr, "-> busy channel %f\n", time);
    }
  }

  // Write mex output arguments.
  janus_rx_mex_write_output_arguments(nargout, pargout, simple_rx, packet, state);

  // Cleanup.
  janus_rx_mex_free(cli_options, params, simple_rx, packet, state, carrier_sensing);
}
