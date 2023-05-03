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

#ifndef JANUS_ERROR_H_INCLUDED_
#define JANUS_ERROR_H_INCLUDED_

//! No error.
#define JANUS_ERROR_NONE              0
//! Resource not found.
#define JANUS_ERROR_NOT_FOUND        -1
#define JANUS_ERROR_FILE             -2
//! Stream error.
#define JANUS_ERROR_STREAM           -3
#define JANUS_ERROR_OTHER           -10
//! End of stream.
#define JANUS_ERROR_EOS             -11
//! Buffer overrun.
#define JANUS_ERROR_OVERRUN         -12
//! Buffer underrun.
#define JANUS_ERROR_UNDERRUN        -13
//! Packet cargo errors
#define JANUS_ERROR_CARGO_SIZE      -14
#define JANUS_ERROR_CARGO_CORRUPTED -15
//! Fields encoding/decoding error
#define JANUS_ERROR_FIELDS          -16

#endif
