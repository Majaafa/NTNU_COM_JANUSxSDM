#include <stdio.h>
#include <string.h>
#include <janus/janus.h>
#include <janus/defaults.h>
#include <janus/utils/dec2bin.h>
#include <janus/utils/memory.h>

#define TEST_CMP(a, b) \
  fprintf(stderr, "-> " #a " == " #b ": %s\n", (a == b) ? "OK" : "FAILED");

void
print_bin(const janus_uint8_t* bfr, unsigned size)
{
  janus_uint8_t bits[8];
  unsigned i;
  unsigned j;

  for (i = 0; i < size; ++i)
  {
    janus_utils_dec2bin_byte(bfr[i], bits);

    for (j = 0; j < 8; ++j)
      printf("%u ", bits[j]);
    printf("| ");
  }

  printf("\n");
}

int
main(int argc, char** argv)
{
  unsigned desired_cargo_size = 480;
  unsigned cargo_size = 0;
  janus_uint8_t* cargo = JANUS_UTILS_MEMORY_ALLOCA(janus_uint8_t, desired_cargo_size);
  double einterval1, einterval2;
  unsigned i, j = 0, k;
  janus_packet_t pkt = 0;

  janus_app_fields_t app_data_fields = janus_app_fields_new();
  janus_app_fields_add_field(app_data_fields, "Station_Identifier", "85");
  janus_app_fields_add_field(app_data_fields, "Parameter Set Identifier", "1");

  for (i = 0; i < desired_cargo_size; i += 10)
  {
    char sub_string[11];
    int inc = (desired_cargo_size - i < 10) ? desired_cargo_size - i : 10;
    j += inc;
    cargo[i] = ' ';
    if (inc > 5)
    {
      for (k = 1; k < inc - 4; k++)
        cargo[i + k] = '_';
      cargo[i + inc - 4] = ' ';
      sprintf(sub_string, "%3d", j);
      strncpy(cargo + i + inc - 3, sub_string, inc);
    }
    else
    {
      for (k = 1; k < inc; k++)
        cargo[i + k] = '_';
    }
  }

  pkt = janus_packet_new(1);

  // Set fields singularly to maximum values.

  janus_packet_set_version(pkt, 15);
  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);
  janus_packet_set_version(pkt, 0);

  janus_packet_set_mobility(pkt, 1);
  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);
  janus_packet_set_mobility(pkt, 0);

  janus_packet_set_tx_rx(pkt, 1);
  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);
  janus_packet_set_tx_rx(pkt, 0);

  janus_packet_set_forward(pkt, 1);
  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);
  janus_packet_set_forward(pkt, 0);

  janus_packet_set_class_id(pkt, 255);
  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);
  janus_packet_set_class_id(pkt, 0);

  janus_packet_set_app_type(pkt, 63);
  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);
  janus_packet_set_app_type(pkt, 0);

  janus_packet_set_tx_reservation_time(pkt, 600.0, &einterval1, &einterval2);
  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);
  janus_packet_unset_tx_interval(pkt);

  janus_packet_set_tx_repeat_interval(pkt, 31557600.0, &einterval1, &einterval2);
  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);
  janus_packet_unset_tx_interval(pkt);

  janus_packet_set_application_data(pkt, 0x3FFFFFF);
  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);
  janus_packet_set_application_data(pkt, 0);

  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);

  // Set all fields together.
  janus_packet_set_version(pkt, 15);
  janus_packet_set_mobility(pkt, 0);
  janus_packet_set_tx_rx(pkt, 0);
  janus_packet_set_forward(pkt, 1);
  janus_packet_set_class_id(pkt, 0);
  janus_packet_set_app_type(pkt, 63);
  janus_packet_set_tx_reservation_time(pkt, 600.0, &einterval1, &einterval2);
  janus_packet_set_application_data(pkt, 0x2AAAAAA);
  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);

  // Dump packet
  printf("\n");
  janus_packet_dump(pkt);

  // Clear packet
  janus_packet_set_version(pkt, 0);
  janus_packet_set_mobility(pkt, 0);
  janus_packet_set_tx_rx(pkt, 0);
  janus_packet_set_forward(pkt, 0);
  janus_packet_set_class_id(pkt, 0);
  janus_packet_set_app_type(pkt, 0);
  janus_packet_unset_tx_interval(pkt);
  janus_packet_set_application_data(pkt, 0);
  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);

  // JANUS Reference Implementation Packet
  printf("\nJANUS Reference Implementation Packet\n");

  janus_packet_set_version(pkt, JANUS_VERSION);
  janus_packet_set_mobility(pkt, 0);
  janus_packet_set_tx_rx(pkt, 1);
  janus_packet_set_forward(pkt, 0);
  janus_packet_set_class_id(pkt, JANUS_RI_CLASS_ID);
  janus_packet_set_app_type(pkt, 0);
  janus_packet_set_tx_repeat_interval(pkt, 33889.13362415, &einterval1, &einterval2);
  janus_packet_set_cargo(pkt, cargo, desired_cargo_size);
  janus_packet_set_application_data_fields(pkt, app_data_fields);
  janus_packet_set_crc(pkt);
  print_bin(janus_packet_get_bytes(pkt), 8);

  // Dump packet
  printf("\n");
  janus_packet_dump(pkt);

  janus_app_fields_free(app_data_fields);

  JANUS_UTILS_MEMORY_ALLOCA_FREE(cargo);

  return 0;
}
