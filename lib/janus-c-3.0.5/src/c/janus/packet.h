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

#ifndef JANUS_PACKET_H_INCLUDED_
#define JANUS_PACKET_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>
#include <janus/codec/fields.h>

//! @ingroup PACKET
//! @{

//! Packet reservation time or repeat interval lookup result valus.
enum janus_packet_interval_lookup_result
{
  //! Exact time.
  JANUS_PACKET_EXACT_TIME,
  //! Within 5% tollerance of the interval step.
  JANUS_PACKET_APPROXIMATED_TIME,
  //! Between two values.
  JANUS_PACKET_BETWEEN_TWO_VALUES,
  //! Below the minimum value.
  JANUS_PACKET_ERROR_MIN,
  //! Beyond the maximum value.
  JANUS_PACKET_ERROR_MAX
};

//! Packet object.
typedef struct janus_packet* janus_packet_t;

//! Create a packet object.
//! @return packet object.
JANUS_EXPORT janus_packet_t
janus_packet_new(janus_uint8_t verbose);

//! Reset packet.
//! @param pkt packet object.
JANUS_EXPORT void
janus_packet_reset(janus_packet_t pkt);

//! Free packet.
//! @param pkt packet object.
JANUS_EXPORT void
janus_packet_free(janus_packet_t pkt);

//! Get packet bytes.
//! @param pkt packet object.
//! @return packet bytes. The size of the returned array is
//! JANUS_MIN_PKT_SIZE.
JANUS_EXPORT const janus_uint8_t*
janus_packet_get_bytes(const janus_packet_t pkt);

//! Set packet bytes.
//! @param pkt packet object.
//! @param bytes array of bytes of size JANUS_MIN_PKT_SIZE.
JANUS_EXPORT void
janus_packet_set_bytes(janus_packet_t pkt, const janus_uint8_t* bytes);

//! Assign to the packet object fields the most relevant values for the
//! receiver encoded into the basic packet.
//! @param pkt packet object.
JANUS_EXPORT void
janus_packet_base_decode(janus_packet_t pkt);

//! Set packet's version.
//! @param pkt packet object.
//! @param version version.
JANUS_EXPORT void
janus_packet_set_version(janus_packet_t pkt, janus_uint8_t version);

//! Get packet's version.
//! @param pkt packet object.
//! @return version.
JANUS_EXPORT janus_uint8_t
janus_packet_get_version(const janus_packet_t pkt);

//! Set packet's mobility flag.
//! @param pkt packet object.
//! @param mobility mobility flag.
JANUS_EXPORT void
janus_packet_set_mobility(janus_packet_t pkt, janus_uint8_t mobility);

//! Get packet's mobility flag.
//! @param pkt packet object.
//! @return mobility flag.
JANUS_EXPORT janus_uint8_t
janus_packet_get_mobility(const janus_packet_t pkt);

//! Set packet's schedule flag.
//! @param pkt packet object.
//! @param tx_rx schedule flag.
JANUS_EXPORT janus_uint8_t
janus_packet_get_schedule(const janus_packet_t pkt);

//! Set packet's TX/RX flag.
//! @param pkt packet object.
//! @param tx_rx TX/RX flag.
JANUS_EXPORT void
janus_packet_set_tx_rx(janus_packet_t pkt, janus_uint8_t tx_rx);

//! Get packet's TX/RX flag.
//! @param pkt packet object.
//! @return TX/RX flag.
JANUS_EXPORT janus_uint8_t
janus_packet_get_tx_rx(const janus_packet_t pkt);

//! Set packet's forwarding capability flag.
//! @param pkt packet object.
//! @param forward forwarding capability flag.
JANUS_EXPORT void
janus_packet_set_forward(janus_packet_t pkt, janus_uint8_t forward);

//! Get packet's forwarding capability flag.
//! @param pkt packet object.
//! @return forwarding capability flag.
JANUS_EXPORT janus_uint8_t
janus_packet_get_forward(const janus_packet_t pkt);

//! Set packet's class identifier.
//! @param pkt packet object.
//! @param class_id class identifier.
JANUS_EXPORT void
janus_packet_set_class_id(janus_packet_t pkt, janus_uint8_t class_id);

//! Get packet's class identifier.
//! @param pkt packet object.
//! @return class identifier.
JANUS_EXPORT janus_uint8_t
janus_packet_get_class_id(const janus_packet_t pkt);

//! Decodes packet's class identifier.
//! @param pkt packet object.
//! @return class identifier.
JANUS_EXPORT janus_uint8_t
janus_packet_decode_class_id(janus_packet_t pkt);

//! Set packet's application type.
//! @param pkt packet object.
//! @param app_type application type (between 0 and 64).
JANUS_EXPORT void
janus_packet_set_app_type(janus_packet_t pkt, janus_uint8_t app_type);

//! Get packet's application type.
//! @param pkt packet object.
//! @return application type (between 0 and 64).
JANUS_EXPORT janus_uint8_t
janus_packet_get_app_type(janus_packet_t pkt);

//! Decodes packet's application type.
//! @param pkt packet object.
//! @return application type (between 0 and 64).
JANUS_EXPORT janus_uint8_t
janus_packet_decode_app_type(const janus_packet_t pkt);

//! Unset packet's transmit time.
//! To be call only before calling janus_packet_set_application_data().
//! @param pkt packet object.
JANUS_EXPORT void
janus_packet_unset_tx_interval(janus_packet_t pkt);

//! Try to set packet's transmit reservation time.
//! Lookup the index associated with a given transmit reservation time
//! (in seconds) in the reservation time table.
//! If the desired value or an approximation (within a tollerance) is
//! available in the lookup table, then found value is returned and
//! its index is set in the packet.
//! If the desired value is not available in the lookup table, the two
//! nearest values (one lower and one higher than the desired value)
//! are returned, and no index is set in the packet.
//! If desired value is too low, lowest table value is returned, and
//! no index is set in the packet.
//! If desired value is too high, highest table value is returned, and
//! no index is set in the packet.
//! N.B.: call only before calling janus_packet_set_application_data()
//! or janus_packet_set_application_data_fields().
//! N.B.: the functions janus_packet_set_tx_reservation_time and
//! janus_packet_set_tx_repeat_interval write the same packet fields.
//! @param pkt packet object.
//! @param dvalue desired transmit reservation time (in seconds).
//! @param evalue1 found table value, if result is JANUS_PACKET_EXACT_TIME
//! or JANUS_PACKET_APPROXIMATED_TIME; highest table value lower than
//! dvalue, if result is JANUS_PACKET_BETWEEN_TWO_VALUES; lowest table
//! value, if result is JANUS_PACKET_ERROR_MIN; highest table value, if
//! result is JANUS_PACKET_ERROR_MAX.
//! @param evalue2 lowest table value higher than dvalue, if result is
//! JANUS_PACKET_BETWEEN_TWO_VALUES, 0.0 otherwise.
//! @return result of set operation. It can be:
//! JANUS_PACKET_EXACT_TIME, if dvalue is found in the lookup table and
//! its index is set in the packet;
//! JANUS_PACKET_APPROXIMATED_TIME, if an approximation of dvalue is found
//! in the lookup table and its index is set in the packet;
//! JANUS_PACKET_BETWEEN_TWO_VALUES, if dvalue is between two values (both
//! are returned) and no index is set in the packet;
//! JANUS_PACKET_ERROR_MIN, if dvalue is too low (lowest lookup table value
//! is returned) and no index is set in the packet;
//! JANUS_PACKET_ERROR_MAX, if dvalue is too high (highest lookup table
//! value is returned) and no index is set in the packet.
JANUS_EXPORT enum janus_packet_interval_lookup_result
janus_packet_set_tx_reservation_time(janus_packet_t pkt, const double dvalue, double* evalue1, double* evalue2);

//! Try to set packet's transmit repeat interval.
//! Lookup the index associated with a given transmit repeat interval
//! (in seconds) in the repeat interval table.
//! If the desired value or an approximation (within a tollerance) is
//! available in the lookup table, then found value is returned and
//! its index is set in the packet.
//! If the desired value is not available in the lookup table, the two
//! nearest values (one lower and one higher than the desired value)
//! are returned, and no index is set in the packet.
//! If desired value is too low, lowest table value is returned, and
//! no index is set in the packet.
//! If desired value is too high, highest table value is returned, and
//! no index is set in the packet.
//! N.B.: call only before calling janus_packet_set_application_data()
//! or janus_packet_set_application_data_fields().
//! N.B.: the functions janus_packet_set_tx_reservation_time and
//! janus_packet_set_tx_repeat_interval write the same packet fields.
//! @param pkt packet object.
//! @param dvalue desired transmit repeat interval (in seconds).
//! @param evalue1 found table value, if result is JANUS_PACKET_EXACT_TIME
//! or JANUS_PACKET_APPROXIMATED_TIME; highest table value lower than
//! dvalue, if result is JANUS_PACKET_BETWEEN_TWO_VALUES; lowest table
//! value, if result is JANUS_PACKET_ERROR_MIN; highest table value, if
//! result is JANUS_PACKET_ERROR_MAX.
//! @param evalue2 lowest table value higher than dvalue, if result is
//! JANUS_PACKET_BETWEEN_TWO_VALUES, 0.0 otherwise.
//! @return result of set operation. It can be:
//! JANUS_PACKET_EXACT_TIME, if dvalue is found in the lookup table and
//! its index is set in the packet;
//! JANUS_PACKET_APPROXIMATED_TIME, if an approximation of dvalue is found
//! in the lookup table and its index is set in the packet;
//! JANUS_PACKET_BETWEEN_TWO_VALUES, if dvalue is between two values (both
//! are returned) and no index is set in the packet;
//! JANUS_PACKET_ERROR_MIN, if dvalue is too low (lowest lookup table value
//! is returned) and no index is set in the packet;
//! JANUS_PACKET_ERROR_MAX, if dvalue is too high (highest lookup table
//! value is returned) and no index is set in the packet.
JANUS_EXPORT enum janus_packet_interval_lookup_result
janus_packet_set_tx_repeat_interval(janus_packet_t pkt, double dvalue, double* evalue1, double* evalue2);

//! Get packet's transmit repeat interval or reservation time.
//! @param pkt packet object.
//! @return transmit repeat interval or reservation time in seconds.
JANUS_EXPORT double
janus_packet_get_tx_interval(const janus_packet_t pkt, janus_uint8_t* reservation_repeat_flag);

//! Decodes packet's application data into the fields.
//! @param pkt packet object.
//! @return application data.
JANUS_EXPORT janus_int64_t
janus_packet_decode_application_data(janus_packet_t pkt);

//! Set packet's application data from application fields.
//! @param pkt packet object.
JANUS_EXPORT janus_uint8_t
janus_packet_encode_application_data(janus_packet_t pkt);

//! Set packet's application data.
//! @param pkt packet object.
//! @param application_data application data.
JANUS_EXPORT janus_uint8_t
janus_packet_set_application_data(const janus_packet_t pkt, janus_uint64_t app_data);

//! Get packet's application data.
//! @param pkt packet object.
//! @return application data.
JANUS_EXPORT janus_uint64_t
janus_packet_get_application_data(const janus_packet_t pkt, janus_uint8_t* app_data_size);

//! Get packet's application data size.
//! @param pkt packet object.
//! @return number of bits of application data block.
JANUS_EXPORT unsigned
janus_packet_get_application_data_size(const janus_packet_t pkt);

//! Set packet's application data fields. Call only after calling
//! janus_packet_set_class_id(), janus_packet_set_app_type(),
//! janus_packet_set_tx_reservation_time() and/or
//! janus_packet_set_tx_repeat_interval().
//! @param pkt packet object.
//! @param app_fields application fields.
//! @return error code (0 = no error, -1 = error).
JANUS_EXPORT int
janus_packet_set_application_data_fields(janus_packet_t pkt, janus_app_fields_t app_fields);

//! Get packet's application data fields.
//! @param pkt packet object.
//! @param app_fields returned application fields.
//! @return error code (0 = no error, -1 = error).
JANUS_EXPORT int
janus_packet_get_application_data_fields(const janus_packet_t pkt, janus_app_fields_t app_fields);

//! Set packet's cargo size.
//! @param pkt packet object.
//! @param cargo_size new size in bytes value
JANUS_EXPORT void
janus_packet_set_cargo_size(const janus_packet_t pkt, unsigned cargo_size);

//! Get packet's optional cargo size.
//! @param pkt packet object.
//! @return optional cargo size.
JANUS_EXPORT unsigned
janus_packet_get_cargo_size(const janus_packet_t pkt);

//! Get size assigned to the packet.
//! @param pkt packet object.
//! @return cargo size.
JANUS_EXPORT unsigned
janus_packet_get_desired_cargo_size(const janus_packet_t pkt);

//! Set packet's optional cargo.
//! @param pkt packet object.
//! @param cargo optional cargo.
//! @param cargo_size optional cargo size.
//! @return JANUS_ERROR_CARGO_SIZE in case of exceeding maximum cargo size
JANUS_EXPORT int
janus_packet_set_cargo(janus_packet_t pkt, janus_uint8_t* cargo, unsigned cargo_size);

//! Get packet's optional cargo.
//! @param pkt packet object.
//! @return optional cargo.
JANUS_EXPORT janus_uint8_t*
janus_packet_get_cargo(const janus_packet_t pkt);

//! Encode packet's cargo.
//! @param pkt packet object.
//! @return JANUS_ERROR_CARGO_SIZE in case of exceeding maximum cargo size
JANUS_EXPORT int
janus_packet_encode_cargo(janus_packet_t pkt);

//! Decode packet's cargo.
//! @param pkt packet object.
//! @return error code.
JANUS_EXPORT int
janus_packet_decode_cargo(const janus_packet_t pkt);

//! Compute and set packet's CRC.
//! @param pkt packet object.
//! @return computed CRC.
JANUS_EXPORT janus_uint8_t
janus_packet_set_crc(janus_packet_t pkt);

//! Get packet's CRC.
//! @param pkt packet object.
JANUS_EXPORT janus_uint8_t
janus_packet_get_crc(const janus_packet_t pkt);

//! Get packet's CRC validity.
//! @param pkt packet object.
//! @return 1 if CRC is valid, 0 otherwise.
JANUS_EXPORT janus_uint8_t
janus_packet_get_crc_validity(const janus_packet_t pkt);

//! Set packet's validity.
//! @param pkt packet object.
//! @param val new validity value
JANUS_EXPORT void
janus_packet_set_validity(janus_packet_t pkt, janus_uint8_t val);

//! Get packet's validity.
//! @param pkt packet object.
//! @return 1 if packet CRC is valid, 2 if cargo validd, 0 otherwise.
JANUS_EXPORT janus_uint8_t
janus_packet_get_validity(const janus_packet_t pkt);

//! Set packet's error status of cargo decoding.
//! @param pkt packet object.
JANUS_EXPORT void
janus_packet_set_cargo_error(janus_packet_t pkt, int val);

//! Get packet's error status of cargo decoding.
//! @param pkt packet object.
//! @return 0 if packet is valid
JANUS_EXPORT int
janus_packet_get_cargo_error(const janus_packet_t pkt);

//! Print packet contents to standard output.
//! @param pkt packet object.
JANUS_EXPORT void
janus_packet_dump(const janus_packet_t pkt);

//! @}

#endif
