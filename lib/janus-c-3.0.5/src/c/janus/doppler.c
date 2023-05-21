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
// Authors: Giovanni Zappa, Luigi Elia D'Amaro                            *
//*************************************************************************

// ISO C headers.
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// FFTW3 headers.
#include <fftw3.h>

// JANUS headers.
#include <janus/complex.h>
#include <janus/doppler.h>
#include <janus/utils/memory.h>
#include <janus/utils/hamming_window.h>
#include <janus/utils/quad_fitn.h>

typedef JANUS_FFTW_PREFIX(complex) janus_fftw_complex_t;
typedef JANUS_FFTW_PREFIX(plan) janus_fftw_plan_t;

#define FFTW_COMPLEX_MUL_REAL(x, y, r) \
{                                      \
  r[0] = x[0] * y;                     \
  r[1] = x[1] * y;                     \
}

#define FFTW_COMPLEX_ABS_SQR(x) (x[0] * x[0] + x[1] * x[1])

// Zero padding factor (odd number).
#define DOPPLER_INT_NPOINT 9

struct janus_doppler
{
  janus_pset_t pset;
  janus_uint8_t enable;
  janus_real_t* freq_list;
  unsigned nchip;
  unsigned bband_fs;
  janus_real_t max_speed;
  janus_real_t chip_size;
  janus_real_t c;
  unsigned size;
  unsigned hf_npoint;
  unsigned cpad;
  janus_real_t res_freq;
  unsigned fs_shift;
  janus_real_t* fvalues;
  janus_complex_t* preamble;
  janus_real_t* gammas;
  janus_real_t* psd;

  //! FFT Plan.
  janus_fftw_plan_t fft_plan;
  //! Buffer to store input of FFT.
  janus_fftw_complex_t* fft_in;
  //! Buffer to store result of FFT.
  janus_fftw_complex_t* fft_out;
};

static int
compare(const void* a, const void* b)
{
  janus_real_t ra = *(janus_real_t*)a;
  janus_real_t rb = *(janus_real_t*)b;
  return (ra > rb) - (ra < rb);
}

static inline unsigned
shift_index(int index, unsigned cpad)
{
  if (index < 0) index += cpad;
  if (index >= (int)cpad) index -= cpad;
  return index;
}

janus_doppler_t
janus_doppler_new(janus_uint8_t enable, janus_pset_t pset, janus_real_t freq_list[], unsigned nchip, unsigned bband_fs, janus_real_t max_speed)
{
  janus_doppler_t doppler = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_doppler, 1);

  doppler->enable = enable;
  doppler->pset = pset;
  doppler->nchip = nchip;
  doppler->bband_fs = bband_fs;
  doppler->chip_size = pset->chip_dur * bband_fs;
  doppler->max_speed = max_speed;
  doppler->c = JANUS_REAL_CONST(1540.0);

  if (enable)
  {
    janus_real_t max_cfactor = doppler->c / (doppler->c - max_speed);
    unsigned lacc = (unsigned)JANUS_CEIL(pset->chip_dur * bband_fs * (nchip * doppler->c / (doppler->c - max_speed) - (nchip - 1) * doppler->c / (doppler->c + max_speed)));
    janus_real_t max_abs_freq = 0;
    unsigned cpad_div2;
    unsigned i;

    doppler->freq_list = JANUS_UTILS_MEMORY_NEW_ZERO(janus_real_t, nchip);
    memcpy(doppler->freq_list, freq_list, nchip * sizeof(janus_real_t));
    doppler->size = (unsigned)JANUS_CEIL(doppler->nchip * doppler->chip_size * max_cfactor) + 1;
    doppler->hf_npoint = (DOPPLER_INT_NPOINT / 2);
    doppler->cpad = JANUS_MAX((unsigned)JANUS_ROUND((janus_real_t)(DOPPLER_INT_NPOINT + 3) * bband_fs * pset->chip_dur / JANUS_REAL_CONST(2.0)), lacc);
    doppler->res_freq = (janus_real_t)bband_fs / doppler->cpad;
    for (i = 0; i < nchip; ++i)
    {
      janus_real_t freq = JANUS_FABS(freq_list[i]);
      if (max_abs_freq < freq)
        max_abs_freq = freq;
    }
    doppler->fs_shift = (unsigned)JANUS_CEIL((pset->cfreq + max_abs_freq + 2 / pset->chip_dur) * max_speed / doppler->c / doppler->res_freq);

    doppler->fvalues = JANUS_UTILS_MEMORY_NEW_ZERO(janus_real_t, doppler->cpad);
    cpad_div2 = doppler->cpad / 2;
    for (i = 0; i < doppler->cpad; i++)
    {
      doppler->fvalues[i] = i * doppler->res_freq;
      if (i >= cpad_div2)
        doppler->fvalues[i] -= bband_fs;
    }

    doppler->preamble = JANUS_UTILS_MEMORY_NEW_ZERO(janus_complex_t, doppler->size);
    doppler->gammas = JANUS_UTILS_MEMORY_NEW_ZERO(janus_real_t, doppler->nchip);
    doppler->psd = JANUS_UTILS_MEMORY_NEW_ZERO(janus_real_t, doppler->cpad);

    doppler->fft_in = (janus_fftw_complex_t*)JANUS_FFTW_PREFIX(malloc)(sizeof(janus_fftw_complex_t) * doppler->cpad);
    memset(doppler->fft_in, 0, doppler->cpad * sizeof(janus_fftw_complex_t));
    doppler->fft_out = (janus_fftw_complex_t*)JANUS_FFTW_PREFIX(malloc)(sizeof(janus_fftw_complex_t) * doppler->cpad);
    memset(doppler->fft_out, 0, doppler->cpad * sizeof(janus_fftw_complex_t));
    doppler->fft_plan = JANUS_FFTW_PREFIX(plan_dft_1d)(doppler->cpad, doppler->fft_in, doppler->fft_out, FFTW_FORWARD, FFTW_MEASURE);
  }

  return doppler;
}

void
janus_doppler_free(janus_doppler_t doppler)
{
  if (doppler->enable)
  {
    JANUS_UTILS_MEMORY_FREE(doppler->psd);
    JANUS_UTILS_MEMORY_FREE(doppler->gammas);
    JANUS_UTILS_MEMORY_FREE(doppler->preamble);
    JANUS_UTILS_MEMORY_FREE(doppler->fvalues);
    JANUS_UTILS_MEMORY_FREE(doppler->freq_list);

    JANUS_FFTW_PREFIX(destroy_plan)(doppler->fft_plan);
    JANUS_FFTW_PREFIX(free)(doppler->fft_in);
    JANUS_FFTW_PREFIX(free)(doppler->fft_out);
    //JANUS_FFTW_PREFIX(cleanup)();
  }

  JANUS_UTILS_MEMORY_FREE(doppler);
}

unsigned
janus_doppler_execute(janus_doppler_t doppler, janus_utils_fifo_t fifo, janus_real_t* gamma, janus_real_t* speed)
{
  *gamma = JANUS_REAL_CONST(1.0);
  *speed = JANUS_REAL_CONST(0.0);

  if (doppler->enable == 0)
  {
    unsigned skip_size = (unsigned)JANUS_ROUND(doppler->nchip * doppler->chip_size);
    return skip_size;
  }

  if (janus_utils_fifo_get_size(fifo) >= doppler->size * sizeof(janus_complex_t))
  {
    unsigned count, i;
    unsigned gamma_count = 0; // Number of valid estimated gammas.

    janus_utils_fifo_peek(fifo, doppler->preamble, doppler->size * sizeof(janus_complex_t));
    
#if 0
    for (i = 0; i < ((doppler->size > 10) ? 10 : doppler->size); i++)
      printf("%+.15f %+.15f\n", doppler->preamble[i][0], doppler->preamble[i][1]);
#endif

    // Compute Doppler estimator quadratic interpolation.
    for (count = 0; count < doppler->nchip; ++count)
    {
      unsigned c_start = JANUS_MAX(0, (unsigned)JANUS_ROUND(doppler->bband_fs * doppler->pset->chip_dur * count * doppler->c / (doppler->c + doppler->max_speed)));
      unsigned c_end = JANUS_MIN(doppler->size, (unsigned)JANUS_ROUND(doppler->bband_fs * doppler->pset->chip_dur * (count + 1) * doppler->c / (doppler->c - doppler->max_speed)));
      janus_real_t a, b;

      // FFT.
      unsigned chip_size = c_end - c_start + 1;

#ifdef JANUS_FFTW_SINGLE

#define FFTW_COMPLEX_CAST(x, y) \
{                               \
  x[0] = y[0];                  \
  x[1] = y[1];                  \
}

      for (i = 0; i < chip_size; ++i)
      { 
        FFTW_COMPLEX_CAST(doppler->fft_in[i], doppler->preamble[c_start + i]);
      }
#else

#if 0
      printf("chip: %d\n", count);
      for (i = 0; i < chip_size; ++i)
      { 
        printf("abs: %f%s\n", janus_complex_abs(doppler->preamble[c_start + i]), (janus_complex_abs(doppler->preamble[c_start + i]) < 0.2 ? " *" : ""));
      }
#endif

      memcpy(doppler->fft_in, doppler->preamble + c_start, chip_size * sizeof(janus_complex_t));
#endif
      
      // Hamming window.
      {
        janus_real_t* ham_win = JANUS_UTILS_MEMORY_ALLOCA(janus_real_t, chip_size);
        janus_utils_hamming_window(chip_size, ham_win);

        for (i = 0; i < chip_size; ++i)
        {
          FFTW_COMPLEX_MUL_REAL(doppler->fft_in[i], ham_win[i], doppler->fft_in[i]);
        }

        JANUS_UTILS_MEMORY_ALLOCA_FREE(ham_win);
      }

      JANUS_FFTW_PREFIX(execute)(doppler->fft_plan);

      for (i = 0; i < doppler->cpad; ++i)
      {
        doppler->psd[i] = FFTW_COMPLEX_ABS_SQR(doppler->fft_out[i]);
      }

      {
        unsigned f_idx = (unsigned)JANUS_ROUND(JANUS_FMOD(doppler->freq_list[count] + doppler->bband_fs, doppler->bband_fs) / doppler->res_freq);
#if 0
        fprintf(stderr, "freq_list[count] %11.3f fvalues %11.3f f_idx %u \n", doppler->freq_list[count], doppler->fvalues[f_idx], f_idx);
#endif 

        // Compute the peak position.
        int start_index = f_idx - doppler->fs_shift - doppler->hf_npoint;
        janus_real_t max_psd = JANUS_REAL_CONST(0.0);
        int f_peak_idx = 0;
        int index;
        janus_real_t x[DOPPLER_INT_NPOINT];
        janus_real_t y[DOPPLER_INT_NPOINT];
        int f_offset;

        for (index = start_index; index != start_index + 2 * doppler->fs_shift + DOPPLER_INT_NPOINT; ++index)
        {
          unsigned curr_index = shift_index(index, doppler->cpad);
          janus_real_t curr_psd = doppler->psd[curr_index];
          if (max_psd < curr_psd)
          {
            max_psd = curr_psd;
            f_peak_idx = curr_index;
          }
        }

        f_offset = f_peak_idx - doppler->hf_npoint;

        // Computes the interpolation coefficients.
        for (i = 0; i != DOPPLER_INT_NPOINT; ++i)
        {
          unsigned curr_index = shift_index(i + f_offset, doppler->cpad);
          x[i] = doppler->fvalues[curr_index] - doppler->freq_list[count];
          y[i] = doppler->psd[curr_index];
        }

        janus_utils_quad_fitn(x, y, DOPPLER_INT_NPOINT, &a, &b);
      }

      if (a < 0)
      {
        janus_real_t est_f_offset = -b / (JANUS_REAL_CONST(2.0) * a);
        janus_real_t gamma_chip = JANUS_REAL_CONST(1.0) + est_f_offset / (doppler->freq_list[count] + doppler->pset->cfreq);
        janus_real_t speed_chip = doppler->c - doppler->c * gamma_chip;
        if (JANUS_FABS(speed_chip) < doppler->max_speed)
        {
          // fprintf(stderr, "chip = %02d, freq = %-3.1f, err = %-1.3f\n", gamma_count, doppler->fvalues[f_peak_idx], est_f_offset);
          doppler->gammas[gamma_count++] = gamma_chip;
        }
      }
    }

    if (gamma_count)
    {
      qsort(doppler->gammas, gamma_count, sizeof(janus_real_t), compare);

      // Median value as Matlab.
      if (gamma_count % 2 == 0)
      {
        *gamma = (doppler->gammas[gamma_count / 2 - 1] + doppler->gammas[gamma_count / 2]) / JANUS_REAL_CONST(2.0);
      }
      else
      {
        *gamma = doppler->gammas[gamma_count / 2];
      }
      *speed = doppler->c - doppler->c * (*gamma);
    }

    return (unsigned)JANUS_CEIL(doppler->nchip * doppler->chip_size / (*gamma));
  }

  return 0;
}

void
janus_doppler_reset(janus_doppler_t doppler)
{
  if (doppler->enable)
  {
    memset(doppler->fft_in, 0, doppler->cpad * sizeof(janus_fftw_complex_t));
  }
}

janus_real_t
janus_doppler_get_min_gamma(janus_doppler_t doppler)
{
  return JANUS_REAL_CONST(1.0) - doppler->max_speed / doppler->c;
}
