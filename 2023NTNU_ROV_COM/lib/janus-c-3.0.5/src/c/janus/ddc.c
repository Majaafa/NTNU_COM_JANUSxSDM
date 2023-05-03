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
#include <string.h>

// FFTW3 headers.
#include <fftw3.h>

// JANUS headers.
#include <janus/types.h>
#include <janus/utils/fir.h>
#include <janus/utils/gcd.h>
#include <janus/complex.h>
#include <janus/constants.h>
#include <janus/ddc.h>
#include <janus/utils/memory.h>

#define FIR_FILTER_ORDER 255
#define FFT_FILTER_SIZE 2048

typedef JANUS_FFTW_PREFIX(complex) janus_fftw_complex_t;
typedef JANUS_FFTW_PREFIX(plan) janus_fftw_plan_t;

#define FFTW_COMPLEX_MUL_REAL(x, y, r) \
{                                      \
  r[0] = x[0] * y;                     \
  r[1] = x[1] * y;                     \
}

struct janus_ddc
{
  // Passband sampling frequency (Hz).
  unsigned pband_fs;
  // Passband sampling period (s).
  janus_real_t pband_ts;
  // Baseband sampling frequency (Hz).
  unsigned bband_fs;
  // Center frequency (Hz).
  unsigned cfreq;
  // Bandwidth (Hz).
  unsigned bwidth;
  // Ratio.
  unsigned ratio;

  //! Buffer for baseband data.
  janus_complex_t* bband;
  //! Size of baseband buffer in samples.
  unsigned bband_size;
  //! Size of baseband buffer in bytes.
  unsigned bband_size_bytes;
  
  // FIR filter FFT.
  janus_fftw_complex_t* filter;

  // fixme: find the meaning of these fields.
  unsigned off;
  unsigned f1;
  janus_complex_t phase1;
  unsigned phase_idx;
  janus_complex_t e[FFT_FILTER_SIZE];

  unsigned phase_per;

  //! Modulator last phasor.
  janus_complex_t phasor;
  //! Phasor increment.
  janus_complex_t inc;

  janus_real_t bfr[FFT_FILTER_SIZE];

  //! FFT Plan.
  janus_fftw_plan_t fft_plan;
  //! Buffer to store result of FFT.
  janus_fftw_complex_t fft[FFT_FILTER_SIZE];
  //! Inverse FFT Plan.
  janus_fftw_plan_t ifft_plan;
  //! Buffer to store result of IFFT.
  janus_fftw_complex_t ifft[FFT_FILTER_SIZE];
};

janus_ddc_t
janus_ddc_new(unsigned pband_fs, unsigned cfreq, unsigned bwidth)
{
  unsigned ratio, i;

  // Allocate and initialize object.
  janus_ddc_t ddc = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_ddc, 1);
  ddc->pband_fs = pband_fs;
  ddc->pband_ts = JANUS_REAL_CONST(1.0) / pband_fs;
  ddc->cfreq = cfreq;
  ddc->bwidth = bwidth;

  // Compute ratio and make it a power of two. The ratio allows 40%
  // more bandwidth than the nyquist frequency.
  bwidth = (unsigned)(bwidth * 1.40);
  ratio = (unsigned)(JANUS_LOG2(JANUS_FLOOR(pband_fs / (JANUS_REAL_CONST(2.0) * bwidth))));
  if (ratio == 0)
  {
    JANUS_UTILS_MEMORY_FREE(ddc);
    return NULL;
  }

  ddc->ratio = 1 << ratio;
  ddc->bband_fs = pband_fs / ddc->ratio;

  ddc->bband_size = (FFT_FILTER_SIZE / 2) / ddc->ratio;
  ddc->bband_size_bytes = sizeof(janus_complex_t) * ddc->bband_size;
  ddc->bband = (janus_complex_t*)malloc(ddc->bband_size_bytes);

  // fixme: add proper comments.
  ddc->off = cfreq / ddc->ratio;
  ddc->f1 = cfreq - ddc->off;
  janus_complex_jexp(-JANUS_HP_2_PI * (FFT_FILTER_SIZE / 2) * ddc->f1 * ddc->pband_ts, ddc->phase1);
  
  // Compute FIR filter.
  {
    janus_utils_fir_t fir = janus_utils_fir_new(FIR_FILTER_ORDER, (janus_real_t)bwidth / pband_fs);

    ddc->filter = (janus_fftw_complex_t*)JANUS_FFTW_PREFIX(malloc)(sizeof(janus_fftw_complex_t) * FFT_FILTER_SIZE);
    memset(ddc->filter, 0, sizeof(janus_fftw_complex_t) * FFT_FILTER_SIZE);

    for (i = 0; i < fir->coefs_n; ++i)
    {
      janus_complex_t tmp;
      janus_complex_jexp(JANUS_HP_2_PI * ddc->off * i * ddc->pband_ts, tmp);
      FFTW_COMPLEX_MUL_REAL(tmp, fir->coefs[i], ddc->filter[i]);
    }

    janus_utils_fir_free(fir);
  }

  // Take the FFT.
  {
    janus_fftw_plan_t plan = JANUS_FFTW_PREFIX(plan_dft_1d)(FFT_FILTER_SIZE, ddc->filter, ddc->filter,
                                                            FFTW_FORWARD, FFTW_ESTIMATE);
    JANUS_FFTW_PREFIX(execute)(plan);
    JANUS_FFTW_PREFIX(destroy_plan)(plan);
  }

  // Find periodicity of phase demodulation.
  ddc->phase_per = pband_fs / janus_utils_gcd((FFT_FILTER_SIZE / 2) * ddc->f1, pband_fs);
  ddc->phase_idx = 0;
  
  // Find periodicity of outer demodulation.
  {
    unsigned outer_fs = pband_fs / ddc->ratio;
    janus_hiprecision_t outer_ts = JANUS_HIPRECISION_CONST(1.0) / outer_fs;
    janus_hiprecision_t phase_inc = -JANUS_HP_2_PI * ddc->off * outer_ts;
    janus_complex_new(JANUS_COS(phase_inc), JANUS_SIN(phase_inc), ddc->inc);
    janus_complex_new(JANUS_REAL_CONST(1.0), JANUS_REAL_CONST(0.0), ddc->phasor);
  }

  // fixme: add proper comments.
  for (i = 0; i < FFT_FILTER_SIZE; ++i)
  {
    janus_complex_jexp(-JANUS_HP_2_PI * ddc->f1 * i * ddc->pband_ts, ddc->e[i]);
  }

  // Create FFT plans.
  memset(ddc->fft, 0, sizeof(janus_fftw_complex_t) * FFT_FILTER_SIZE);
  ddc->fft_plan = JANUS_FFTW_PREFIX(plan_dft_1d)(FFT_FILTER_SIZE, ddc->fft, ddc->fft,
                                                 FFTW_FORWARD, FFTW_MEASURE);
  ddc->ifft_plan = JANUS_FFTW_PREFIX(plan_dft_1d)(FFT_FILTER_SIZE, ddc->ifft, ddc->ifft,
                                                  FFTW_BACKWARD, FFTW_MEASURE);
  
  return ddc;
}

void
janus_ddc_free(janus_ddc_t ddc)
{
  JANUS_FFTW_PREFIX(destroy_plan)(ddc->fft_plan);
  JANUS_FFTW_PREFIX(destroy_plan)(ddc->ifft_plan);
  JANUS_FFTW_PREFIX(free)(ddc->filter);
  JANUS_FFTW_PREFIX(cleanup)();

  free(ddc->bband);
  JANUS_UTILS_MEMORY_FREE(ddc);
}

unsigned
janus_ddc_get_output_fs(janus_ddc_t ddc)
{
  return ddc->bband_fs;
}

unsigned
janus_ddc_get_ratio(janus_ddc_t ddc)
{
  return ddc->ratio;
}

unsigned
janus_ddc_execute(janus_ddc_t ddc, janus_real_t* pband, janus_complex_t* bband)
{
  janus_hiprecision_t val = -JANUS_HP_2_PI * ddc->phase_idx * (FFT_FILTER_SIZE / 2) * ddc->f1 * ddc->pband_ts;
  janus_complex_t phase = {0};
  int i, j;
  
  janus_complex_jexp(val, phase);
  
  // Old.
  for (i = 0; i < FFT_FILTER_SIZE / 2; ++i)
  {
    // ddc->fft[i] = ddc->e[i] * phase * ddc->bfr[i]
    janus_real_t amc = (ddc->e[i][0] * phase[0]);
    janus_real_t bmd = (ddc->e[i][1] * phase[1]);
    ddc->fft[i][0] = (amc - bmd) * ddc->bfr[i];
    ddc->fft[i][1] = ((ddc->e[i][0] + ddc->e[i][1]) * (phase[0] + phase[1]) - amc - bmd) * ddc->bfr[i];
  }

  // New.
  for (i = 0; i < FFT_FILTER_SIZE / 2; ++i)
  {
    // ddc->fft[i + (FFT_FILTER_SIZE / 2)] = ddc->e[i + (FFT_FILTER_SIZE/2)] * phase * pband[i]
    janus_real_t amc = (ddc->e[i + (FFT_FILTER_SIZE/2)][0] * phase[0]);
    janus_real_t bmd = (ddc->e[i + (FFT_FILTER_SIZE/2)][1] * phase[1]);
    ddc->fft[i + (FFT_FILTER_SIZE / 2)][0] = (amc - bmd) * pband[i];
    ddc->fft[i + (FFT_FILTER_SIZE / 2)][1] = ((ddc->e[i + (FFT_FILTER_SIZE/2)][0] + ddc->e[i + (FFT_FILTER_SIZE/2)][1]) * (phase[0] + phase[1]) - amc - bmd) * pband[i];
    
    ddc->bfr[i] = pband[i];
  }

  JANUS_FFTW_PREFIX(execute)(ddc->fft_plan);

  for (i = 0; i < FFT_FILTER_SIZE; ++i)
  {
    // ddc->ifft[i] = ddc->fft[i] * ddc->filter[i]
    janus_fftw_real_t amc = (ddc->fft[i][0] * ddc->filter[i][0]);
    janus_fftw_real_t bmd = (ddc->fft[i][1] * ddc->filter[i][1]);
    ddc->ifft[i][0] = amc - bmd;
    ddc->ifft[i][1] = (ddc->fft[i][0] + ddc->fft[i][1]) * (ddc->filter[i][0] + ddc->filter[i][1]) - amc - bmd;
  }

  JANUS_FFTW_PREFIX(execute)(ddc->ifft_plan);

  j = 0;
  {
    janus_complex_t p;
    janus_complex_assign(ddc->phasor, p);
    for (i = FFT_FILTER_SIZE / 2; i < FFT_FILTER_SIZE; i += ddc->ratio)
    {
      janus_real_t amc = ddc->ifft[i][0] / FFT_FILTER_SIZE * ddc->phase1[0];
      janus_real_t bmd = ddc->ifft[i][1] / FFT_FILTER_SIZE * ddc->phase1[1];
      janus_real_t amc2 = (bmd - amc) * p[0];
      janus_real_t bmd2 = (amc + bmd - (ddc->ifft[i][0] + ddc->ifft[i][1]) / FFT_FILTER_SIZE * (ddc->phase1[0] + ddc->phase1[1])) * p[1];
      janus_real_t amc3 = p[0] * ddc->inc[0];
      janus_real_t bmd3 = p[1] * ddc->inc[1];
      janus_real_t apbmcpd3 = (p[0] + p[1]) * (ddc->inc[0] + ddc->inc[1]);

      // bband[j] = -ddc->ifft[i] / FFT_FILTER_SIZE * ddc->phase1 * p
      bband[j][0] = amc2 - bmd2;
      bband[j][1] = (bmd + bmd - (ddc->ifft[i][0] + ddc->ifft[i][1]) / FFT_FILTER_SIZE * (ddc->phase1[0] + ddc->phase1[1])) * (p[0] + p[1]) - amc2 - bmd2;

      // p = p * ddc->inc
      p[0] = amc3 - bmd3;
      p[1] = apbmcpd3 - amc3 - bmd3;

      ++j;
    }

    janus_complex_mul_real(p, JANUS_REAL_CONST(1.0) / janus_complex_abs(p), p); 
    janus_complex_assign(p, ddc->phasor);
  }

  ddc->phase_idx = (ddc->phase_idx + 1) % ddc->phase_per;

  return ddc->bband_size;
}

unsigned
janus_ddc_get_output_sample_count(janus_ddc_t ddc)
{
  return ddc->bband_size;
}

unsigned
janus_ddc_get_input_sample_count(janus_ddc_t ddc)
{
  return FFT_FILTER_SIZE / 2;
}
