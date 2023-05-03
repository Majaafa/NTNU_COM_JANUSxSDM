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

#define SCHEDULE_NAME "Schedule"
#define STATION_ID    "Station_Identifier"

#define CONTACT_TYPE_NAME  "Type"
#define TYPE_OFFSET(i) (100 + 97 * (i - 1))

#define CONTACT_DEPTH_NAME "Depth"
#define DEPTH_OFFSET(i) (100 + 97 * (i - 1) + 4)

#define NoEC_NAME          "Extra_Contacts"

#define USERID_NAME "User_ID"
#define USERID_OFFSET(i) ((i > 0 ? (100 + 97 * (i - 1) + 14) : 0))

#define LATITUDE_NAME "Latitude"
#define LATITUDE_OFFSET(i) ((i > 0 ? (100 + 97 * (i - 1) + 14) : 0) + 30)

#define LONGITUDE_NAME "Longitude"
#define LONGITUDE_OFFSET(i) ((i > 0 ? (100 + 97 * (i - 1) + 60) : 54))

#define SPEED_NAME "Speed"
#define SPEED_OFFSET(i) ((i > 0 ? (100 + 97 * (i - 1) + 76) : 79))

#define NAVIGATIONAL_STATUS_NAME "Navigation_Status"
#define NAVIGATIONAL_STATUS_OFFSET(i) ((i > 0 ? (100 + 97 * (i - 1) + 93) : 96))

#define TRUE_HEADING_NAME "True_Heading"

#define COG_NAME "CoG"
#define COG_OFFSET(i) ((i > 0 ? (100 + 97 * (i - 1) + 84) : 87))

char *type_str(int contact_type)
{
    static char *type_str_[] ={"Nuclear submarine", "AIP submarine", "Conventional submarine",
                               "AUV", "Ship", "Airplane", "UAV", "USV", "Buoy", "Bottom node"};
    return ((contact_type < 10) ? type_str_[contact_type] : "n.a.");
}

int str_to_type(char *type_str)
{
    static char *type_str_[] ={"Nuclear submarine", "AIP submarine", "Conventional submarine",
                               "AUV", "Ship", "Airplane", "UAV", "USV", "Buoy", "Bottom node"};
    int i, vehicle_type = 15;

    for (i = 0; i < 10; i++) {
        if (strcmp(type_str, type_str_[i]) == 0) {
            vehicle_type = i;
            break;
        }
    }
    return vehicle_type;
}

char *status_str(int status)
{
    static char *status_str_[] = {
        "Under way - using engine",
        "At anchor",
        "Not under command",
        "Restricted manoeuvrability",
        "Constrained by draught",
        "Moored",
        "Aground",
        "Engaged in fishin",
        "Under way - sailing",
        "For future use",
        "For future use",
        "Power-driven vessel towing astern",
        "Power-driven vessel pushing ahead",
        "For future use",
        "AIS-SART"};
    return ((status < 15) ? status_str_[status] : "Undefined/default");
}

int str_to_status(char *status_str)
{
    static char *status_str_[] = {
        "Under way - using engine",
        "At anchor",
        "Not under command",
        "Restricted manoeuvrability",
        "Constrained by draught",
        "Moored",
        "Aground",
        "Engaged in fishin",
        "Under way - sailing",
        "For future use",
        "For future use",
        "Power-driven vessel towing astern",
        "Power-driven vessel pushing ahead",
        "For future use",
        "AIS-SART"};
    int i, status = 15;

    for (i = 0; i < 15; i++) {
        if (strcmp(status_str, status_str_[i]) == 0) {
            status = i;
            break;
        }
    }
    return status;
}

uint32_t contact_depth(uint32_t depth)
{
    if (depth < 700) {
        return depth;
    } else if (depth < 730) {
        return 700 + (depth - 700) * 10;
    } else if (depth < 830) {
        return 1000 + (depth - 730) * 20;
    } else if (depth < 950) {
        return 3000 + (depth - 830) * 25;
    } else if (depth < 1024) {
        return 6000 + (depth - 950) * 75;
    }
}

uint32_t encode_depth(uint32_t depth)
{
    if (depth < 700) {
        return depth;
    } else if (depth < 1000) {
        return 700 + floor((depth - 700) * 1./10);
    } else if (depth < 3000) {
        return 730 + floor((depth - 1000) * 1./20);
    } else if (depth < 6000) {
        return 830 + floor((depth - 3000) * 1./25);
    } else if (depth <= 11400) {
        return 950 + floor((depth - 6000) * 1./75);
    } else {
       return 1022;
    }
}

double contact_speed(uint32_t speed)
{
    if (speed < 200) {
        return speed * 0.1;
    } else if (speed < 250) {
        return 20 + (speed - 200) * 1;
    } else if (speed < 254) {
        return 70 + (speed - 250) * 5;
    } else {
        return 86.0;
    }
}

uint32_t encode_speed(double speed)
{
    if (speed < 0) {
        return 255;
    } else if (speed < 20) {
        return speed * 10;
    } else if (speed < 70) {
        return 200 + (speed - 20);
    } else if (speed < 86) {
        return 250 + (speed - 70.) / 5; 
    } else {
        return 254;
    }
}

static janus_uint16_t contact_type, depth;

JANUS_PLUGIN_EXPORT int
app_data_decode(janus_uint64_t app_data, janus_uint8_t app_data_size, unsigned* cargo_size, janus_app_fields_t app_fields)
{
  char value[16];
  janus_uint16_t schedule, station_id, noc;

  // Schedule (8 bits)
  schedule = (janus_uint8_t)((app_data >> 26) & (0xFFU));
  sprintf(value, "%u", schedule);
  janus_app_fields_add_field(app_fields, SCHEDULE_NAME, value);

  // Station_Identifier (8 bits)
  station_id = (janus_uint8_t)((app_data >> 17) & (0xFFU));
  sprintf(value, "%u", station_id);
  janus_app_fields_add_field(app_fields, STATION_ID, value);

  // Number of contacts (3 bits).
  noc = (unsigned)(app_data & (0x7U));
  sprintf(value, "%u", noc);
  janus_app_fields_add_field(app_fields, NoEC_NAME, value);

  // Contact 1 Type (4)
  contact_type = (janus_uint8_t)((app_data >> 13) & (0x0FU));
  sprintf(value, "%s", type_str(contact_type));
  janus_app_fields_add_field(app_fields, CONTACT_TYPE_NAME, value);
  
  // Contact 1 Depth (10 bits)
  depth = (janus_uint16_t)((app_data >> 3) & (0x3FFU));
  sprintf(value, "%u", contact_depth(depth));
  janus_app_fields_add_field(app_fields, CONTACT_DEPTH_NAME, value);

  *cargo_size = (100 + 97 * (noc) + 16 + 7) / 8;
  return 0;
}

JANUS_PLUGIN_EXPORT int
app_data_encode(unsigned desired_cargo_size, janus_app_fields_t app_fields, janus_uint8_t app_data_size, unsigned* cargo_size, janus_uint64_t* app_data)
{
  uint64_t field;
  int i;
  *app_data = 0;
  
  // Cargo Size (9 bits).
  /* *app_data = (desired_cargo_size & 0x1FFU); */
  *cargo_size = desired_cargo_size;

  for (i = 0; i != app_fields->field_count; ++i) {
      if (strcmp(app_fields->fields[i].name, SCHEDULE_NAME) == 0) {
          field = atoi(app_fields->fields[i].value) & 0xFFU;
          field <<= 26;
          *app_data |= field;
      }
      if (strcmp(app_fields->fields[i].name, STATION_ID) == 0) {
          field = atoi(app_fields->fields[i].value) & 0xFFU;
          field <<= 17;
          *app_data |= field;
      }
      if (strcmp(app_fields->fields[i].name, NoEC_NAME) == 0) {
          field = atoi(app_fields->fields[i].value) & 0x7U;
          *app_data |= field;
      }
      if (i < 8) {
          if (strcmp(app_fields->fields[i].name, CONTACT_TYPE_NAME) == 0) {
              field = str_to_type(app_fields->fields[i].value);
              field <<= 13;
              *app_data |= field;
          }
          if (strcmp(app_fields->fields[i].name, CONTACT_DEPTH_NAME) == 0) {
              if (strcmp(app_fields->fields[i].value, "n.a.") == 0) {
                  field = 1023;
              } else {
                  field = encode_depth(atoi(app_fields->fields[i].value));
              }
              field <<= 3;
              *app_data |= field;
          }
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
  janus_uint8_t i, noc, field_len, status;
  double lat0, lon0, lat, lon;
  janus_uint8_t lat_flag = 0, lon_flag = 0;

  field = janus_crc_16(cargo, cargo_size - 2, 0);
  field = SWAP32_BY_BITSIZE(field, 16);
  barr2int(cargo, (cargo_size - sizeof(janus_uint16_t)) * 8, &crc, 0, 16);

  if (crc != field) {
      janus_app_fields_add_field(*app_fields, CRC_NAME, "0");
      return -1;
  }
  janus_app_fields_add_field(*app_fields, CRC_NAME, "1");

  noc = (cargo_size * 8) / 97;
 
  for (i = 0; i < noc; i++) {
      if (i > 0) {
          field = 0;
          barr2int(cargo, TYPE_OFFSET(i), &field, 0, 4);
          field >>= 4;
          sprintf(value, "%s", type_str(field));
          janus_app_fields_add_field(*app_fields, CONTACT_TYPE_NAME, value);
          field = 0;
          barr2int(cargo, DEPTH_OFFSET(i), &field, 0, 10);
          field = swap(field, (10+7)/8);
          field >>= 6;
          sprintf(value, "%u", contact_depth(field));
          janus_app_fields_add_field(*app_fields, CONTACT_DEPTH_NAME, value);
      }
      field = 0;
      barr2int(cargo, USERID_OFFSET(i), &field, 0, 30);
      field = swap(field, (30+7)/8);
      field >>= 2;
      sprintf(value, "%u", field);
      janus_app_fields_add_field(*app_fields, USERID_NAME, value);

      field = 0;
      field_len = i == 0 ? 24 : 16;
      barr2int(cargo, LATITUDE_OFFSET(i), &field, 0, field_len);
      field = swap(field, (field_len+7)/8);
      if (field == 0x800000 || lat_flag) {
          sprintf(value, "n.a.");
          if (i == 0) {
              lat_flag = 1;
          }
      } else {
          if (field_len == 16) {
              lat = (double)((int16_t)field*90)/8388607;
          } else {
              lat = (double)(field*90)/8388607;
          }
          if (lat > 90) lat = lat - 180;
          if (i == 0) {
              lat0 = lat;
          } else {
              lat = lat0 + lat;
          }
          sprintf(value, "%f", lat);
      }
      janus_app_fields_add_field(*app_fields, LATITUDE_NAME, value);

      field = 0;
      field_len = i == 0 ? 25 : 16;
      barr2int(cargo, LONGITUDE_OFFSET(i), &field, 0, field_len);
      field = swap(field, (field_len+7)/8);
      field >>= (field_len % 8) ? 8 - (field_len % 8) : 0;
      if (field == 0x1000000 || lon_flag) {
          sprintf(value, "n.a.");
          if (i == 0) {
              lon_flag = 1;
          }
      } else {
          if (field_len == 16) {
              lon = ((double)((int16_t)field*90))/8388607;
          } else {
              lon = ((double)(field*90))/8388607;
          }
          if (lon > 180) lon = lon - 360;
          if (i == 0) {
              lon0 = lon;
          } else {
              lon = lon0 + lon;
          }
          sprintf(value, "%f", lon);
      }
      janus_app_fields_add_field(*app_fields, LONGITUDE_NAME, value);
      
      field = 0;
      barr2int(cargo, SPEED_OFFSET(i), &field, 0, 8);
      if (field == 255) {
          sprintf(value, "n.a.");
      } else if (field == 254) {
          sprintf(value, ">= 86");
      } else {
          sprintf(value, "%f", contact_speed(field));
      }
      janus_app_fields_add_field(*app_fields, SPEED_NAME, value);

      field = 0;
      barr2int(cargo, NAVIGATIONAL_STATUS_OFFSET(i), &field, 0, 4);
      status = field >> 4;
      
      field = 0;
      barr2int(cargo, COG_OFFSET(i), &field, 0, 9);
      field = swap(field, (9+7)/8);
      field >>= 7;
      if (field == 511) {
          sprintf(value, "n.a.");
      } else {
          sprintf(value, "%f", field * 0.705);
      }
      switch (status) {
      case 1: /* At anchor */
      case 5: /* Moored */
      case 6: /* Aground */
          janus_app_fields_add_field(*app_fields, TRUE_HEADING_NAME, value);
          janus_app_fields_add_field(*app_fields, COG_NAME, "n.a.");
          break;
      default:
          janus_app_fields_add_field(*app_fields, TRUE_HEADING_NAME, "n.a.");
          janus_app_fields_add_field(*app_fields, COG_NAME, value);
      }

      sprintf(value, "%s", status_str(status));
      janus_app_fields_add_field(*app_fields, NAVIGATIONAL_STATUS_NAME, value);
  }
  
  return 0;
}

JANUS_PLUGIN_EXPORT int
cargo_encode(janus_app_fields_t app_fields, janus_uint8_t** cargo, unsigned* cargo_size)
{
  uint32_t field;
  double f;
  unsigned i = 0, j = 0, field_len;
  janus_uint8_t l1 = 0, l2 = 0;
  uint8_t extra_contacts = 0;
  double lat0 = 0, lon0 = 0, lat = 0, lon = 0;
  janus_uint8_t lat_flag = 0, lon_flag = 0, status;
  double th = 0, cog = 0;
  
  for (j = 0; j != app_fields->field_count; ++j) {
      if (strcmp(app_fields->fields[j].name, NoEC_NAME) == 0) {
          extra_contacts = atoi(app_fields->fields[j].value);
      }
  }
  *cargo_size = (100 + 97 * extra_contacts + 16 + 7) / 8;
  *cargo = JANUS_UTILS_MEMORY_REALLOC(*cargo, janus_uint8_t, *cargo_size);

  for (j = 0; j != app_fields->field_count; ++j) {
      if (i > 0) {
          if (strcmp(app_fields->fields[j].name, CONTACT_TYPE_NAME) == 0) {
              field = str_to_type(app_fields->fields[j].value);
              field <<= 4;
              int2barr(*cargo, TYPE_OFFSET(i), &field, 0, 4);
          }
          if (strcmp(app_fields->fields[j].name, CONTACT_DEPTH_NAME) == 0) {
              if (strcmp(app_fields->fields[j].value, "n.a.") == 0) {
                  field = 1023;
              } else {
                  field = encode_depth(atoi(app_fields->fields[j].value));
              }
              field <<= 6;
              field = swap(field, (10+7)/8);
              int2barr(*cargo, DEPTH_OFFSET(i), &field, 0, 10);
          }
      }
      if (strcmp(app_fields->fields[j].name, USERID_NAME) == 0) {
          field = atoi(app_fields->fields[j].value) & 0x3fffffffU;
          field <<= 2;
          field = swap(field, (30+7)/8);
          int2barr(*cargo, USERID_OFFSET(i), &field, 0, 30);
      }
      if (strcmp(app_fields->fields[j].name, LATITUDE_NAME) == 0) {
          if (strcmp(app_fields->fields[j].value, "n.a.") == 0 || lat_flag) {
              if (i == 0) {
                  lat_flag = 1;
              }
              field = 0x800000;
          } else {
              f = atof(app_fields->fields[j].value);
              if (i == 0) {
                  lat = lat0 = f;
              } else {
                  lat = f - lat0;
              }
              field = round(lat * 8388607 / 90);
          }
          field_len = i == 0 ? 24 : 16;
          field = swap(field, (field_len+7)/8);
          int2barr(*cargo, LATITUDE_OFFSET(i), &field, 0, field_len);
      }

      if (strcmp(app_fields->fields[j].name, LONGITUDE_NAME) == 0) {
          if (strcmp(app_fields->fields[j].value, "n.a.") == 0 || lon_flag) {
              if (i == 0) {
                  lon_flag = 1;
              }
              field = 0x1000000;
          } else {
              f = atof(app_fields->fields[j].value);
              if (i == 0) {
                  lon = lon0 = f;
              } else {
                  lon = f - lon0;
              }
              field = round(lon * 8388607 / 90);
          }
          printf("f = %f, field = %d\n", f, field);
          field_len = i == 0 ? 25 : 16;
          field <<= (field_len % 8) ? 8 - (field_len % 8) : 0;
          field = swap(field, (field_len+7)/8);
          int2barr(*cargo, LONGITUDE_OFFSET(i), &field, 0, field_len);
      }

      if (strcmp(app_fields->fields[j].name, TRUE_HEADING_NAME) == 0) {
          th = atof(app_fields->fields[j].value);
      }

      if (strcmp(app_fields->fields[j].name, COG_NAME) == 0) {
          cog = atof(app_fields->fields[j].value);
      }

      if (strcmp(app_fields->fields[j].name, SPEED_NAME) == 0) {
          if (strcmp(app_fields->fields[j].value, "n.a.") == 0) {
              field = 255;
          } else if (strcmp(app_fields->fields[j].value, ">= 86") == 0) {
              field = 254;
          } else {
              field = encode_speed(atof(app_fields->fields[j].value));
          }
          int2barr(*cargo, SPEED_OFFSET(i), &field, 0, 8);
      }      
      
      if (strcmp(app_fields->fields[j].name, NAVIGATIONAL_STATUS_NAME) == 0) {
          field = status = str_to_status(app_fields->fields[j].value);
          field <<= 4;
          int2barr(*cargo, NAVIGATIONAL_STATUS_OFFSET(i), &field, 0, 4);
          switch (status) {
          case 1: /* At anchor */
          case 5: /* Moored */
          case 6: /* Aground */
              field = round(th/0.705);
              break;
          default:
              field = round(cog/0.705);
          }
          if (field < 0 || field > 511) field = 511;
          field <<= 7;
          field = swap(field, (9+7)/8);
          int2barr(*cargo, COG_OFFSET(i), &field, 0, 9);
          
          i++;
      }
  }

  field = janus_crc_16(*cargo, (*cargo_size) - sizeof(janus_uint16_t), 0);
  field = SWAP32_BY_BITSIZE(field, 16);
  int2barr(*cargo, (*cargo_size - sizeof(janus_uint16_t)) * 8, &field, 0, 16);
  return 0;
}
