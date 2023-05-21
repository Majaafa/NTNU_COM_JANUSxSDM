// ISO C headers.
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

// JANUS headers.
#include <janus/defaults.h>
#include <janus/codec/codec.h>
#include <janus/error.h>
#include <janus/crc.h>

#include <janus/msb.h>

#define CRC_NAME "Cargo CRC Validity"

#define STATION_ID_LABEL "Station_Identifier"
#define DESTINATION_ID_LABEL "Destination_Identifier"
#define ACK_REQUEST_LABEL "Ack_Request"

#define PAYLOAD_SIZE_LABEL "Payload_Size"
#define PAYLOAD_LABEL "Payload"

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
app_data_decode_destination_id(janus_uint64_t app_data, janus_app_fields_t app_fields)
{
  char name[] = DESTINATION_ID_LABEL;
  char value[4];

  janus_uint8_t station_id = (janus_uint8_t)((app_data >> 10) & (0xFFU));
  sprintf(value, "%u", station_id);

  janus_app_fields_add_field(app_fields, name, value);
}

static inline void
app_data_decode_ack_request(janus_uint64_t app_data, janus_app_fields_t app_fields)
{
  char name[] = ACK_REQUEST_LABEL;
  char value[16];

  janus_uint8_t ack_req = (janus_uint8_t)((app_data >> 9) & (0x1U));
  sprintf(value, "%u", ack_req);

  janus_app_fields_add_field(app_fields, name, value);
}

static inline unsigned
app_data_decode_cargo_size(janus_uint64_t app_data)
{
  return (unsigned)(app_data & (0xFFU));
}

JANUS_PLUGIN_EXPORT int
app_data_decode(janus_uint64_t app_data, janus_uint8_t app_data_size, unsigned* cargo_size, janus_app_fields_t app_fields)
{
  // Station_Identifier (8 bits).
  app_data_decode_station_id(app_data, app_fields);

  // Destination Identifier (8 bits).
  app_data_decode_destination_id(app_data, app_fields);

  // Ack_Request (1 bit).
  app_data_decode_ack_request(app_data, app_fields);
  
  // Cargo Size (8 bits).
  *cargo_size = app_data_decode_cargo_size(app_data);

  return 0;
}

JANUS_PLUGIN_EXPORT int
app_data_encode(unsigned desired_cargo_size, janus_app_fields_t app_fields, janus_uint8_t app_data_size, unsigned* cargo_size, janus_uint64_t* app_data)
{
  uint64_t field;
  int i;
  *app_data = 0;
  
  // Cargo Size (8 bits).
  *app_data = (desired_cargo_size & 0xFFU);
  *cargo_size = desired_cargo_size;

  for (i = 0; i != app_fields->field_count; ++i) {
      if (strcmp(app_fields->fields[i].name, STATION_ID_LABEL) == 0) {
          field = atoi(app_fields->fields[i].value) & 0xFFU;
          field <<= 18;
          *app_data |= field;
      }
      if (strcmp(app_fields->fields[i].name, DESTINATION_ID_LABEL) == 0) {
          field = atoi(app_fields->fields[i].value) & 0xFFU;
          field <<= 10;
          *app_data |= field;
      }
      if (strcmp(app_fields->fields[i].name, ACK_REQUEST_LABEL) == 0) {
          field = atoi(app_fields->fields[i].value) & 0x1U;
          field <<= 9;
          *app_data |= field;
      }
  }
  
  return 0;
}

JANUS_PLUGIN_EXPORT int
cargo_decode(janus_uint8_t* cargo, unsigned cargo_size, janus_app_fields_t* app_fields)
{
  int rv = 0;
  int payload_size = cargo_size - 2;
  janus_uint32_t field = 0;
  janus_uint16_t crc;

  field = janus_crc_16(cargo, cargo_size - 2, 0);
  field = SWAP32_BY_BITSIZE(field, 16);
  barr2int(cargo, (cargo_size - sizeof(janus_uint16_t)) * 8, &crc, 0, 16);

  if (crc != field) {
      janus_app_fields_add_field(*app_fields, CRC_NAME, "0");
      return -1;
  }
  janus_app_fields_add_field(*app_fields, CRC_NAME, "1");

  if (*app_fields == 0)
  {
    *app_fields = janus_app_fields_new();
  }

  char size_string[4];
  sprintf(size_string, "%u", payload_size);

  janus_app_fields_add_field(*app_fields, PAYLOAD_SIZE_LABEL, size_string);

  janus_app_fields_add_blob(*app_fields, PAYLOAD_LABEL, cargo, payload_size);

  return rv;
}

JANUS_PLUGIN_EXPORT int
cargo_encode(janus_app_fields_t app_fields, janus_uint8_t** cargo, unsigned* cargo_size)
{
  int rv = JANUS_ERROR_FIELDS;
  int cargo_size_found = 0;
  janus_uint32_t field = 0;

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
  *cargo_size += sizeof(uint16_t);
  *cargo = JANUS_UTILS_MEMORY_REALLOC(*cargo, janus_uint8_t, *cargo_size);
  field = janus_crc_16(*cargo, (*cargo_size) - sizeof(janus_uint16_t), 0);
  field = SWAP32_BY_BITSIZE(field, 16);
  memcpy(*cargo + *cargo_size * sizeof(janus_uint8_t) - 2, &field, 2);

  return rv;
}
