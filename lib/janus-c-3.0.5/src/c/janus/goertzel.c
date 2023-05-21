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

// ISO C headers.
#include <stdlib.h>
#include <string.h>

// JANUS headers.
#include <janus/goertzel.h>
#include <janus/constants.h>
#include <janus/utils/memory.h>

struct janus_goertzel
{
  //! Sampling frequency.
  unsigned fs;
  //! Number of frequencies.
  unsigned freq_vector_size;
  //! Frequencies.
  janus_real_t* freq_vector;
  //! Number of samples (window length).
  unsigned n;

  //! Constants
  janus_real_t* coeff;
  janus_complex_t* complex_coeff;
  janus_complex_t* complex_coeff_n1;
  
  janus_complex_t* gdft;
};

static inline void
janus_goertzel_common_impl(janus_goertzel_t goertzel, janus_complex_t x[], janus_complex_t r[])
{
  unsigned i, k;
  janus_real_t amc;
  janus_real_t bmd;
    
  for (k = 0; k < goertzel->freq_vector_size; k++)
  {
    janus_complex_t s0 = {0}, s1 = {0}, s2 = {0};
     
    for (i = 0; i < goertzel->n - 1; i++)
    {
      // s0 = x[i] + (coeff * s1) - s2 (*)
      s0[0] = x[i][0] + (s1[0] * goertzel->coeff[k]) - s2[0];
      s0[1] = x[i][1] + (s1[1] * goertzel->coeff[k]) - s2[1];

      // s2 = s1
      s2[0] = s1[0];
      s2[1] = s1[1];

      // s1 = s0
      s1[0] = s0[0];
      s1[1] = s0[1];
    }

    // s0 = x[n-1] + (coeff * s1) - s2     correspond to one extra performing of (*)
    s0[0] = x[goertzel->n - 1][0] + (s1[0] * goertzel->coeff[k]) - s2[0];
    s0[1] = x[goertzel->n - 1][1] + (s1[1] * goertzel->coeff[k]) - s2[1];

    // r = s0 - (s1 * complex_coeff)
    amc = (s1[0] * goertzel->complex_coeff[k][0]);
    bmd = (s1[1] * goertzel->complex_coeff[k][1]);
    r[k][0] = s0[0] - (amc - bmd);
    r[k][1] = s0[1] - ((s1[0] + s1[1]) * (goertzel->complex_coeff[k][0] + goertzel->complex_coeff[k][1]) - amc - bmd);
  }
}

janus_goertzel_t
janus_goertzel_new(unsigned fs, unsigned freq_vector_size, janus_real_t freq_vector[], unsigned n)
{
  janus_goertzel_t goertzel = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_goertzel, 1);
  janus_hiprecision_t* pik_term = JANUS_UTILS_MEMORY_ALLOCA(janus_hiprecision_t, freq_vector_size);
  unsigned k;

  goertzel->fs = fs;
  goertzel->freq_vector_size = freq_vector_size;
  goertzel->freq_vector = JANUS_UTILS_MEMORY_NEW_ZERO(janus_real_t, freq_vector_size);
  memcpy(goertzel->freq_vector, freq_vector, sizeof(janus_real_t) * freq_vector_size);
  goertzel->n = n;

  goertzel->coeff = JANUS_UTILS_MEMORY_NEW_ZERO(janus_real_t, freq_vector_size);
  goertzel->complex_coeff = JANUS_UTILS_MEMORY_NEW_ZERO(janus_complex_t, freq_vector_size);
  goertzel->complex_coeff_n1 = JANUS_UTILS_MEMORY_NEW_ZERO(janus_complex_t, freq_vector_size);
  for (k = 0; k < freq_vector_size; k++)
  {
    pik_term[k] = JANUS_HP_2_PI * freq_vector[k] / fs;                         // pik_term = 2 * pi * target_frequency / fs
    goertzel->coeff[k] = 2.0 * JANUS_HP_COS(pik_term[k]);                      // coeff = 2 * cos(pik_term);
    janus_complex_jexp(-pik_term[k], goertzel->complex_coeff[k]);              // complex_coeff = exp(-i * pik_term)
    janus_complex_jexp(-pik_term[k] * (n-1), goertzel->complex_coeff_n1[k]);   // complex_coeff_n1 = exp(-i * pik_term * (n-1));
  }

  goertzel->gdft = JANUS_UTILS_MEMORY_NEW_ZERO(janus_complex_t, goertzel->freq_vector_size);

  JANUS_UTILS_MEMORY_ALLOCA_FREE(pik_term);

  return goertzel;
}

void
janus_goertzel_execute(janus_goertzel_t goertzel, janus_complex_t x[], janus_complex_t r[])
{
  unsigned k;

  janus_goertzel_common_impl(goertzel, x, goertzel->gdft);

  for (k = 0; k < goertzel->freq_vector_size; k++)
  {
    // complex multiplication substituting the last iteration and correcting the phase for (potentially) non-integer valued frequencies at the same time
    // r = r * exp(-i * pik_term * (n-1)) = r * complex_coeff_n1
    janus_complex_mul(goertzel->gdft[k], goertzel->complex_coeff_n1[k], r[k]);
  }
}

void
janus_goertzel_abs_execute(janus_goertzel_t goertzel, janus_complex_t x[], janus_real_t r[])
{
  unsigned k;

  janus_goertzel_common_impl(goertzel, x, goertzel->gdft);

  for (k = 0; k < goertzel->freq_vector_size; k++)
  {
    r[k] = janus_complex_abs(goertzel->gdft[k]);
  }
}

void
janus_goertzel_free(janus_goertzel_t goertzel)
{
  JANUS_UTILS_MEMORY_FREE(goertzel->gdft);
  JANUS_UTILS_MEMORY_FREE(goertzel->complex_coeff_n1);
  JANUS_UTILS_MEMORY_FREE(goertzel->complex_coeff);
  JANUS_UTILS_MEMORY_FREE(goertzel->coeff);
  JANUS_UTILS_MEMORY_FREE(goertzel->freq_vector);
  JANUS_UTILS_MEMORY_FREE(goertzel);
}
