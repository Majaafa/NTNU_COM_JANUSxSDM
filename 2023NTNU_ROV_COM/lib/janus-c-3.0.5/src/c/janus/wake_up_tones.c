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
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

// JANUS headers.
#include <janus/constants.h>
#include <janus/defaults.h>
#include <janus/complex.h>
#include <janus/wake_up_tones.h>
#include <janus/utils/memory.h>
#include <janus/utils/tukey_window.h>

int
janus_wake_up_tones(janus_ostream_t ostream, unsigned bwidth, janus_real_t chip_dur)
{
  janus_hiprecision_t x = JANUS_HP_PI * bwidth * ostream->ts;
  janus_real_t lt = JANUS_ROUND(chip_dur / ostream->ts);
  unsigned tw_len = (unsigned)(JANUS_WAKE_UP_TONE_CHIP_COUNT * (lt + 1));
  janus_real_t* tw = JANUS_UTILS_MEMORY_NEW(janus_real_t, tw_len);
  janus_hiprecision_t a = 0;
  unsigned i = 0;
  janus_complex_t tmp;

  janus_utils_tukey_window(tw_len, JANUS_TUKEY_RATIO, tw);

  // We could do this in only one loop, but this implementation uses
  // less memory.
  for (i = 0; i < tw_len; ++i)
  {
    int rv = 0;
    a = -x * i + JANUS_HP_PI_2;
    janus_complex_jexp(a, tmp);
    janus_complex_mul_real(tmp, tw[i], tmp);
    if ((rv = janus_ostream_write(ostream, &tmp, 1)) < 0)
    {
      free(tw);
      return rv;
    }
  }

  for (i = 0; i < tw_len; ++i)
  {
    int rv = 0;
    janus_complex_new(0, tw[i], tmp);
    if ((rv = janus_ostream_write(ostream, &tmp, 1)) < 0)
    {
      free(tw);
      return rv;
    }
  }

  for (i = 0; i < tw_len; ++i)
  {
    int rv = 0;
    a = x * i + JANUS_HP_PI_2;
    janus_complex_jexp(a, tmp);
    janus_complex_mul_real(tmp, tw[i], tmp);
    if ((rv = janus_ostream_write(ostream, &tmp, 1)) < 0)
    {
      free(tw);
      return rv;
    }
  }

  free(tw);

  return tw_len * 3;
}
