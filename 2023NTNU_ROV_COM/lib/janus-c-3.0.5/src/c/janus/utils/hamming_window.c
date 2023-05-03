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

// ISO C headers.
#include <math.h>
#include <stdio.h>

// JANUS headers.
#include <janus/janus.h>

void
janus_utils_hamming_window(unsigned points, janus_real_t* window)
{
  unsigned n = points - 1;
  unsigned i = 0;

  for (i = 0; i < points; ++i)
    window[i] = (JANUS_REAL_CONST(0.54) - JANUS_REAL_CONST(0.46) * JANUS_COS(JANUS_2_PI * i / n));
}

void
janus_utils_hamming_window_part(unsigned points, unsigned pmin, unsigned pmax, janus_real_t* window)
{
  unsigned n = points - 1;
  unsigned i = 0;
  unsigned idx = 0;
  for (i = pmin; i < pmax; ++i)
    window[idx++] = (JANUS_REAL_CONST(0.54) - JANUS_REAL_CONST(0.46) * JANUS_COS(JANUS_2_PI * i / n));
}
