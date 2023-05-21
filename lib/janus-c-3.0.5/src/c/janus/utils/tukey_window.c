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

// JANUS headers.
#include <janus/constants.h>
#include <janus/utils/tukey_window.h>

void
janus_utils_tukey_window(unsigned points, janus_real_t ratio, janus_real_t* window)
{
  janus_real_t incr = 1.0 / (points - 1);
  unsigned tl = (unsigned)(ratio * (points - 1) / 2) + 1;
  unsigned th = points - tl;
  unsigned i = 0;

  for (i = 0; i < tl; ++i)
  {
    window[i] = (1 + JANUS_COS(JANUS_PI * (2.0 * incr * i / ratio - 1))) / 2.0;
    window[points - i - 1] = window[i];
  }

  for (i = tl; i < th; ++i)
    window[i] = 1;
}
