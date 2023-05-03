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

#ifndef JANUS_CONSTANTS_H_INCLUDED_
#define JANUS_CONSTANTS_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>

//! Mathematical constant π (pi).
#define JANUS_HP_PI                   JANUS_HIPRECISION_CONST(3.1415926535897932384626433832795)
//! Mathematical constant π (pi) multiplied by 2.
#define JANUS_HP_2_PI                 (JANUS_HIPRECISION_CONST(2.0) * JANUS_HP_PI)
//! Mathematical constant π (pi) divided by 2.
#define JANUS_HP_PI_2                 (JANUS_HP_PI / JANUS_HIPRECISION_CONST(2.0))

//! Mathematical constant π (pi).
#define JANUS_PI                      JANUS_REAL_CONST(3.1415926535897932384626433832795)
//! Mathematical constant π (pi) multiplied by 2.
#define JANUS_2_PI                    (JANUS_REAL_CONST(2.0) * JANUS_PI)
//! Mathematical constant π (pi) divided by 2.
#define JANUS_PI_2                    (JANUS_PI / JANUS_REAL_CONST(2.0))

#endif
