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
// Author: Luigi Elia D'Amaro                                             *
//*************************************************************************

#ifndef JANUS_UTILS_QUAD_FITN_H_INCLUDED_
#define JANUS_UTILS_QUAD_FITN_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/export.h>

//! Computes the a*x^2+b*x parameter of the LSE quadratic passing through n points.
//! @param x Vector value.
//! @param y Vector value.
//! @param n Number of vector points.
//! @param a Polynomial coefficient.
//! @param b Polynomial coefficient.
JANUS_EXPORT void
janus_utils_quad_fitn(const janus_real_t* x, const janus_real_t* y, const unsigned n, janus_real_t* a, janus_real_t* b);

#endif
