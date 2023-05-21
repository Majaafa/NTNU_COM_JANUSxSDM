// ISO C headers.
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>
#include <math.h>

// JANUS headers.
#include <janus/defaults.h>
#include <janus/codec/codec.h>
#include <janus/error.h>
#include <janus/crc.h>

#include <janus/msb.h>

#define CRC_NAME "Cargo CRC Validity"
/* By specification */
/* #define CRC_OFFSET (90) */
#define CRC_OFFSET (96)

#define NATIONALITY_NAME   "Nationality"
#define NATIONALITY_OFFSET (0)

#define LATITUDE_NAME "Latitude"
#define LATITUDE_OFFSET (10)

#define LONGITUDE_NAME "Longitude"
#define LONGITUDE_OFFSET (34)

#define DEPTH_NAME "Depth"
#define DEPTH_OFFSET (59)

#define SPEED_NAME "Speed"
#define SPEED_OFFSET (72)

#define HEADING_NAME "Heading"
#define HEADING_OFFSET (81)

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
  double lat, lon;

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

  barr2int(cargo, LATITUDE_OFFSET, &field, 0, 24);
  field = swap(field, (24+7)/8);
  if (field == 0x800000) {
      sprintf(value, "n.a.");
  } else {
      lat = (double)(field*90)/8388607L;
      if (lat > 90) lat = lat - 180;
      sprintf(value, "%f", lat);
  }
  janus_app_fields_add_field(*app_fields, LATITUDE_NAME, value);

  barr2int(cargo, LONGITUDE_OFFSET, &field, 0, 25); 
  field = swap(field, (25+7)/8);
  field >>= 7;
  if (field == 0x1000000) {
      sprintf(value, "n.a.");
  } else {
      /* field -= 4; */
      lon = (double)(field*90)/8388607L;
      if (lon > 180) lon = lon - 360.0;
      sprintf(value, "%f", lon);
  }
  janus_app_fields_add_field(*app_fields, LONGITUDE_NAME, value);

  field = 0;
  barr2int(cargo, DEPTH_OFFSET, &field, 0, 13);
  field = swap(field, (13+7)/8);
  field >>= 3;
  sprintf(value, "%d", field);
  janus_app_fields_add_field(*app_fields, DEPTH_NAME, value);

  field = 0;
  barr2int(cargo, SPEED_OFFSET, &field, 0, 9);
  field = swap(field, (9+7)/8);
  field >>= 7;
  sprintf(value, "%f", field * 0.1);
  janus_app_fields_add_field(*app_fields, SPEED_NAME, value);

  field = 0;
  barr2int(cargo, HEADING_OFFSET, &field, 0, 9);
  field = swap(field, (9+7)/8);
  field >>= 7;
  if (field == 511) {
      sprintf(value, "n.a.");
  } else {
      sprintf(value, "%f", field * 0.705);
  }
  janus_app_fields_add_field(*app_fields, HEADING_NAME, value);

  return 0;
}

JANUS_PLUGIN_EXPORT int
cargo_encode(janus_app_fields_t app_fields, janus_uint8_t** cargo, unsigned* cargo_size)
{
  uint32_t field;
  double f;
  unsigned i;
  janus_uint8_t l1 = 0, l2 = 0;
  
  *cargo_size = (106 + 7) / 8;
  *cargo = JANUS_UTILS_MEMORY_REALLOC(*cargo, janus_uint8_t, *cargo_size);
  for (i = 0; i != app_fields->field_count; ++i) {
      
      if (strcmp(app_fields->fields[i].name, NATIONALITY_NAME) == 0) {
          l1 = app_fields->fields[i].value[0] - 'A' + 1;
          l2 = app_fields->fields[i].value[1] - 'A' + 1;
          l1 <<= 3; int2barr(*cargo, NATIONALITY_OFFSET, &l1, 0, 5);
          l2 <<= 3; int2barr(*cargo, NATIONALITY_OFFSET + 5, &l2, 0, 5);
      }
      
      if (strcmp(app_fields->fields[i].name, LATITUDE_NAME) == 0) {
          field = 0;
          if (strcmp(app_fields->fields[i].value, "n.a.") == 0) {
              field = 0x800000;
          } else {
              f = atof(app_fields->fields[i].value);
              if (f < 0)
                  field = -round(-f * 8388607 / 90);
              else
                  field = round(f * 8388607 / 90);
          }
          field = swap(field, (24+7)/8);
          int2barr(*cargo, LATITUDE_OFFSET, &field, 0, 24);
      }
      
      if (strcmp(app_fields->fields[i].name, LONGITUDE_NAME) == 0) {
          field = 0;
          if (strcmp(app_fields->fields[i].value, "n.a.") == 0) {
              field = 0x1000000;
          } else {
              f = atof(app_fields->fields[i].value);
              if (f < 0)
                  field = -round(-f * 8388607 / 90);
              else
                  field = round(f * 8388607 / 90);
          }
          field <<= 7;
          field = swap(field, (25+7)/8);
          int2barr(*cargo, LONGITUDE_OFFSET, &field, 0, 25); 
      }
      
      if (strcmp(app_fields->fields[i].name, DEPTH_NAME) == 0) {
          field = atoi(app_fields->fields[i].value);
          field <<= 3;
          field = swap(field, (13+7)/8);
          int2barr(*cargo, DEPTH_OFFSET, &field, 0, 13);
      }

      if (strcmp(app_fields->fields[i].name, SPEED_NAME) == 0) {
          f = atof(app_fields->fields[i].value);
          field = round(f * 10);
          field <<= 7;
          field = swap(field, (9+7)/8);
          int2barr(*cargo, SPEED_OFFSET, &field, 0, 9);
      }
  
      if (strcmp(app_fields->fields[i].name, HEADING_NAME) == 0) {
          f = atof(app_fields->fields[i].value);
          if (strcmp(app_fields->fields[i].value, "n.a.") == 0) {
              field = 511;
          } else {
              field = round(f / 0.705);
              field <<= 7;
          }
          field = swap(field, (9+7)/8);
          int2barr(*cargo, HEADING_OFFSET, &field, 0, 9);
      }
  }
  field = janus_crc_16(*cargo, (*cargo_size) - sizeof(janus_uint16_t), 0);
  field = SWAP32_BY_BITSIZE(field, 16);
  int2barr(*cargo, CRC_OFFSET, &field, 0, 16);
  return 0;
}
