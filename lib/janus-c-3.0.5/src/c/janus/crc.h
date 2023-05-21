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
// Author: Ricardo Martins                                                *
//*************************************************************************

#ifndef JANUS_CRC_H_INCLUDED_
#define JANUS_CRC_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>

#define JANUS_MAX_SIZE_CRC_8  63

//! @ingroup PACKET
//! @{

//! Compute the CRC-8 of an array of bytes.
//! @param data input array.
//! @param data_len input array size.
//! @param crc initial CRC value.
//! @return CRC-8 of byte array.
JANUS_EXPORT janus_uint8_t
janus_crc(const janus_uint8_t* data, unsigned data_len, janus_uint8_t crc);

//! Compute the CRC-8 of a byte.
//! @param byte input byte.
//! @param crc initial CRC value.
//! @return CRC-8 of byte.
JANUS_EXPORT janus_uint8_t
janus_crc_byte(janus_uint8_t byte, janus_uint8_t crc);

//! Compute the CRC-16-IBM of a given data buffer.
//! @param buffer data buffer.
//! @param len data buffer length.
//! @param crc CRC-16-IBM value to update.
//! @return computed CRC-16-IBM.
JANUS_EXPORT janus_uint16_t
janus_crc_16(const janus_uint8_t* data, unsigned data_len, janus_uint16_t crc);

//! Compute the CRC-16-IBM of a given byte.
//! @param byte byte.
//! @param crc CRC-16-IBM value to update.
//! @return computed CRC-16-IBM.
JANUS_EXPORT janus_uint16_t
janus_crc_16_byte(janus_uint8_t byte, janus_uint16_t crc);

//! @}

#endif
