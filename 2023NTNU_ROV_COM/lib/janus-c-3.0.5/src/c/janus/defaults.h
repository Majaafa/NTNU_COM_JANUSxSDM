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

#ifndef JANUS_DEFAULTS_H_INCLUDED_
#define JANUS_DEFAULTS_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>

//! JANUS version.
#define JANUS_VERSION                    3
//! JANUS minimum packet size in bytes.
#define JANUS_MIN_PKT_SIZE               8
//! JANUS minimum packet size size in bits.
#define JANUS_MIN_PKT_SIZE_BITS          (JANUS_MIN_PKT_SIZE * 8)
//! JANUS maximum optional packet cargo size in bytes.
#define JANUS_MAX_PKT_CARGO_SIZE         4096
//! JANUS maximum optional packet cargo size in bits.
#define JANUS_MAX_PKT_CARGO_SIZE_BITS    (JANUS_MAX_PKT_CARGO_SIZE * 8)
//! Convolutional encoder memory in bytes.
#define JANUS_CONV_MEM_SIZE              1
//! Convolutional encoder memory in bits.
#define JANUS_CONV_MEM_SIZE_BITS         (JANUS_CONV_MEM_SIZE * 8)
//! Alphabet size.
#define JANUS_ALPHABET_SIZE              2
//! Time for hardware wake-up (s).
#define JANUS_WUT_GAP                    JANUS_REAL_CONST(0.4)
//! Length of carrier-wave wake-up tones in chips.
#define JANUS_WAKE_UP_TONE_CHIP_COUNT    4
//! Length of prologue and epilogue silence in chips.
#define JANUS_SILENCE_CHIP_COUNT         5
//! CRC-8 polynomial.
#define JANUS_CRC_POLY                   0x07
//! Tukey Window ratio of taper to constant sections.
#define JANUS_TUKEY_RATIO                JANUS_REAL_CONST(0.05)
//! Number of frequencies used by a chip (13 is the standard).
#define JANUS_CHIP_FRQ_COUNT             13
//! Default sequence preamble chip count.
#define JANUS_PREAMBLE_CHIP_COUNT        32
//! Default 32 chip sequence preamble (one bit per chip).
#define JANUS_32_CHIP_SEQUENCE           0xAEC7CD20U
//! Chip oversampling
#define JANUS_PREAMBLE_CHIP_OVERSAMPLING 4

//! Number of class identifiers.
#define JANUS_CLASS_COUNT                256
//! Types of message per class identifier.
#define JANUS_APP_COUNT                  64
//! JANUS Reference Implementation Class Id.
#define JANUS_RI_CLASS_ID                16

#endif
