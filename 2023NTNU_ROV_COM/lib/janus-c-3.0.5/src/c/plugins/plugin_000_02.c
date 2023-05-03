// ISO C headers.
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>

// JANUS headers.
#include <janus/defaults.h>
#include <janus/codec/codec.h>
#include <janus/error.h>
#include <janus/crc.h>

#include <janus/msb.h>

#define CRC_NAME "Cargo CRC Validity"
/* By specification */
/* #define CRC_OFFSET (60) */
#define CRC_OFFSET (64)

#define SURVIVORS_NAME   "Survivors"
#define SURVIVORS_OFFSET (52)
  
#define TEMPERATURE_NAME   "Temperature"
#define TEMPERATURE_OFFSET (46)
  
#define PRESSURE_NAME   "Pressure"
#define PRESSURE_OFFSET (36)
  
#define H2_NAME   "H2"
#define H2_OFFSET (30)

#define CO_NAME   "CO"
#define CO_OFFSET (22)
  
#define CO2_NAME   "CO2"
#define CO2_OFFSET (16)
  
#define O2_NAME   "O2"
#define O2_OFFSET (10)

#define NATIONALITY_NAME  "Nationality"
#define NATIONALITY_OFFSET (0)

#define SCHEDULE "Schedule"
#define STATION_ID "Station_Identifier"
#define DESTINATION_ID "Destination Identifier"

JANUS_PLUGIN_EXPORT int
app_data_decode(janus_uint64_t app_data, janus_uint8_t app_data_size, unsigned* cargo_size, janus_app_fields_t app_fields)
{
  char value[4];
  janus_uint8_t schedule, station_id, destination_id;
  
  // Schedule (8 bits)
  schedule = (janus_uint8_t)((app_data >> 26) & (0xFFU));
  sprintf(value, "%u", schedule);
  janus_app_fields_add_field(app_fields, SCHEDULE, value);

  // Station_Identifier (8 bits)
  station_id = (janus_uint8_t)((app_data >> 17) & (0xFFU));
  sprintf(value, "%u", station_id);
  janus_app_fields_add_field(app_fields, STATION_ID, value);

  // Destination Identifier (8 bits)
  destination_id = (janus_uint8_t)((app_data >> 9) & (0xFFU));
  sprintf(value, "%u", destination_id);
  janus_app_fields_add_field(app_fields, DESTINATION_ID, value);

  // Cargo Size (9 bits).
  *cargo_size = (unsigned)(app_data & (0x1FFU));
  return 0;
}

JANUS_PLUGIN_EXPORT int
app_data_encode(unsigned desired_cargo_size, janus_app_fields_t app_fields, janus_uint8_t app_data_size, unsigned* cargo_size, janus_uint64_t* app_data)
{
  uint64_t field;
  int i;
  *app_data = 0;
  
  // Cargo Size (9 bits).
  *app_data = (desired_cargo_size & 0x1FFU);
  *cargo_size = desired_cargo_size;

  for (i = 0; i != app_fields->field_count; ++i) {
      if (strcmp(app_fields->fields[i].name, SCHEDULE) == 0) {
          field = atoi(app_fields->fields[i].value) & 0xFFU;
          field <<= 26;
          *app_data |= field;
      }
      if (strcmp(app_fields->fields[i].name, STATION_ID) == 0) {
          field = atoi(app_fields->fields[i].value) & 0xFFU;
          field <<= 17;
          *app_data |= field;
      }
      if (strcmp(app_fields->fields[i].name, DESTINATION_ID) == 0) {
          field = atoi(app_fields->fields[i].value) & 0xFFU;
          field <<= 9;
          *app_data |= field;
      }
  }
  
  return 0;
}

JANUS_PLUGIN_EXPORT int
cargo_decode(janus_uint8_t* cargo, unsigned cargo_size, janus_app_fields_t* app_fields)
{
  char value[32];
  janus_uint32_t field = 0;
  janus_uint16_t crc;
  janus_uint8_t l1 = 0, l2 = 0;
  double lon;

  field = janus_crc_16(cargo, cargo_size - 2, 0);
  field = SWAP32_BY_BITSIZE(field, 16);
  barr2int(cargo, CRC_OFFSET, &crc, 0, 16);

  if (crc != field) {
      janus_app_fields_add_field(*app_fields, CRC_NAME, "0");
      return -1;
  }
  janus_app_fields_add_field(*app_fields, CRC_NAME, "1");

  barr2int(cargo, NATIONALITY_OFFSET, &l1, 0, 5); l1 >>= 3;
  barr2int(cargo, NATIONALITY_OFFSET + 5, &l2, 0, 5); l2 >>= 3;
  sprintf(value, "%c%c", 'A' + l1 - 1, 'A' + l2 - 1);
  janus_app_fields_add_field(*app_fields, NATIONALITY_NAME, value);

  field = 0;
  barr2int(cargo, O2_OFFSET, &field, 0, 6);
  field >>= 2;
  sprintf(value, "%f", field * 0.1 + 17);
  janus_app_fields_add_field(*app_fields, O2_NAME, value);
  
  field = 0;
  barr2int(cargo, CO2_OFFSET, &field, 0, 6);
  field >>= 2;
  sprintf(value, "%f", field * 0.1);
  janus_app_fields_add_field(*app_fields, CO2_NAME, value);

  field = 0;
  barr2int(cargo, CO_OFFSET, &field, 0, 8);
  sprintf(value, "%d", field);
  janus_app_fields_add_field(*app_fields, CO_NAME, value);
  
  field = 0;
  barr2int(cargo, H2_OFFSET, &field, 0, 6);
  field >>= 2;
  sprintf(value, "%f", field * 0.1);
  janus_app_fields_add_field(*app_fields, H2_NAME, value);

  field = 0;
  barr2int(cargo, PRESSURE_OFFSET, &field, 0, 10);
  field = swap(field, (10+7)/8);
  field >>= 6;
  sprintf(value, "%f", field * 0.1 + 0.9);
  janus_app_fields_add_field(*app_fields, PRESSURE_NAME, value);
  
  field = 0;
  barr2int(cargo, TEMPERATURE_OFFSET, &field, 0, 6);
  field >>= 2;
  sprintf(value, "%d", field);
  janus_app_fields_add_field(*app_fields, TEMPERATURE_NAME, value);

  field = 0;
  barr2int(cargo, SURVIVORS_OFFSET, &field, 0, 8);
  sprintf(value, "%d", field);
  janus_app_fields_add_field(*app_fields, SURVIVORS_NAME, value);

  return 0;
}

JANUS_PLUGIN_EXPORT int
cargo_encode(janus_app_fields_t app_fields, janus_uint8_t** cargo, unsigned* cargo_size)
{
  uint32_t field;
  double f;
  unsigned i;
  janus_uint8_t l1 = 0, l2 = 0;
  
  *cargo_size = (76 + 7) / 8;
  *cargo = JANUS_UTILS_MEMORY_REALLOC(*cargo, janus_uint8_t, *cargo_size);
  for (i = 0; i != app_fields->field_count; ++i) {
      
      if (strcmp(app_fields->fields[i].name, NATIONALITY_NAME) == 0) {
          l1 = app_fields->fields[i].value[0] - 'A' + 1;
          l2 = app_fields->fields[i].value[1] - 'A' + 1;
          l1 <<= 3; int2barr(*cargo, NATIONALITY_OFFSET, &l1, 0, 5);
          l2 <<= 3; int2barr(*cargo, NATIONALITY_OFFSET + 5, &l2, 0, 5);
      }

      if (strcmp(app_fields->fields[i].name, O2_NAME) == 0) {
          f = atof(app_fields->fields[i].value);
          if ( f >= 17 && f < 23.3) {
              field = round((f - 17) * 10.0);
          } else {
              field = 63;
          }
          field <<= 2;
          int2barr(*cargo, O2_OFFSET, &field, 0, 6);
      }
      
      if (strcmp(app_fields->fields[i].name, CO2_NAME) == 0) {
          f = atof(app_fields->fields[i].value);
          field = round(f * 10);
          field <<= 2;
          int2barr(*cargo, CO2_OFFSET, &field, 0, 6);
      }

      if (strcmp(app_fields->fields[i].name, CO_NAME) == 0) {
          field = atoi(app_fields->fields[i].value);
          int2barr(*cargo, CO_OFFSET, &field, 0, 8);
      }
  
      if (strcmp(app_fields->fields[i].name, H2_NAME) == 0) {
          f = atof(app_fields->fields[i].value);
          field = round(f * 10);
          field <<= 2;
          int2barr(*cargo, H2_OFFSET, &field, 0, 6);
      }

      if (strcmp(app_fields->fields[i].name, PRESSURE_NAME) == 0) {
          f = atof(app_fields->fields[i].value);
          if ( f >= 0.9 && f < 103.2) {
              field = round((f - 0.9) * 10.0);
          } else {
              field = 1023;
          }
          field <<= 6;
          field = swap(field, (10+7)/8);
          int2barr(*cargo, PRESSURE_OFFSET, &field, 0, 10);
      }
  
      if (strcmp(app_fields->fields[i].name, TEMPERATURE_NAME) == 0) {
          field = atoi(app_fields->fields[i].value);
          field <<= 2;
          int2barr(*cargo, TEMPERATURE_OFFSET, &field, 0, 6);
      }

      if (strcmp(app_fields->fields[i].name, SURVIVORS_NAME) == 0) {
          field = atoi(app_fields->fields[i].value);
          int2barr(*cargo, SURVIVORS_OFFSET, &field, 0, 8);
      }
  }

  field = janus_crc_16(*cargo, (*cargo_size) - sizeof(janus_uint16_t), 0);
  field = SWAP32_BY_BITSIZE(field, 16);
  int2barr(*cargo, CRC_OFFSET, &field, 0, 16);
  return 0;
}
