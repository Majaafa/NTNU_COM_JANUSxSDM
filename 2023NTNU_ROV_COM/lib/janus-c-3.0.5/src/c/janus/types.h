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
// Authors: Ricardo Martins, Giovanni Zappa, Luigi Elia D'Amaro           *
//*************************************************************************

#ifndef JANUS_TYPES_H_INCLUDED_
#define JANUS_TYPES_H_INCLUDED_

// ISO C headers.
#include <float.h>
#include <math.h>

#include <janus/config.h>

//! @ingroup TYPES
//! @{

//! Signed 8-bit integer.
typedef signed char        janus_int8_t;
//! Unsigned 8-bit integer.
typedef unsigned char      janus_uint8_t;
//! Signed 16-bit integer.
typedef signed short       janus_int16_t;
//! Unsigned 16-bit integer.
typedef unsigned short     janus_uint16_t;
//! Signed 32-bit integer.
typedef signed int         janus_int32_t;
//! Unsigned 32-bit integer.
typedef unsigned int       janus_uint32_t;
//! Signed 64-bit integer.
typedef signed long long   janus_int64_t;
//! Unsigned 64-bit integer.
typedef unsigned long long janus_uint64_t;

//! Return the maximum of a and b.
#define JANUS_MAX(a, b) (((a) > (b)) ? a : b)
//! Return the minimum of a and b.
#define JANUS_MIN(a, b) (((a) < (b)) ? a : b)

// Do not uncomment next line, use cmake -DJANUS_REAL_SINGLE=1
//#define JANUS_REAL_SINGLE
#if defined(JANUS_REAL_SINGLE)
//! Single precision floating point type.
typedef float janus_real_t;
//! Utility macro to append a suffix to a function call.
#  define JANUS_REAL_SUFFIX(a) a ## f
//! Utility macro to append a suffix to a number literal.
#  define JANUS_REAL_CONST(a)  a ## f
//! Format to read real numbers using the scanf family of functions.
#  define JANUS_REAL_FMT       "%f"
//! Smallest number x such that 1.0 + x != 1.0
#  define JANUS_REAL_EPSILON    FLT_EPSILON
#else
//! Double precision floating point type.
typedef double janus_real_t;
//! Utility macro to append a suffix to a function call.
#  define JANUS_REAL_SUFFIX(a) a
//! Utility macro to append a suffix to a number literal.
#  define JANUS_REAL_CONST(a)  a
//! Format to read real numbers using the scanf family of functions.
#  define JANUS_REAL_FMT       "%lf"
//! Smallest number x such that 1.0 + x != 1.0
#  define JANUS_REAL_EPSILON   DBL_EPSILON
#endif

#if defined(JANUS_FFTW_SINGLE) || defined(JANUS_REAL_SINGLE)
//! Utility macro to prepend a prefix to a FFTW function call or type.
#  define JANUS_FFTW_PREFIX(a) fftwf_ ## a
#  define janus_fftw_real_t float
#else
//! Utility macro to prepend a prefix to a FFTW function call or type.
#  define JANUS_FFTW_PREFIX(a) fftw_ ## a
#  define janus_fftw_real_t double
#endif

//#define JANUS_HIPRECISION_SINGLE
#if defined(JANUS_HIPRECISION_SINGLE)
//! Single precision floating point type.
typedef float janus_hiprecision_t;
//! Utility macro to append a suffix to a function call.
#  define JANUS_HIPRECISION_SUFFIX(a) a ## f
//! Utility macro to append a suffix to a number literal.
#  define JANUS_HIPRECISION_CONST(a)  a ## f
//! Format to read hi precision numbers using the scanf family of functions.
#  define JANUS_HIPRECISION_FMT       "%f"
#else
//! Double precision floating point type.
typedef double janus_hiprecision_t;
//! Utility macro to append a suffix to a function call.
#  define JANUS_HIPRECISION_SUFFIX(a) a
//! Utility macro to append a suffix to a number literal.
#  define JANUS_HIPRECISION_CONST(a)  a
//! Format to read hi precision numbers using the scanf family of functions.
#  define JANUS_HIPRECISION_FMT       "%lf"
#endif

// Microsoft Visual Studio lacks a few ISO C99 functions.
#if __STDC_VERSION__ < 199901L
//! Round v to the nearest integer (double).
#  define round(v)    floor(v + 0.5)
//! Round v to the nearest integer (float).
#  define roundf(v)   floor(v + 0.5f)
//! Return the maximum of a and b (double).
#  define fmax(a, b)  (((a) > (b)) ? a : b)
//! Return the maximum of a and b (float).
#  define fmaxf(a, b) fmax(a, b)
//! Return the minimum of a and b (double).
#  define fmin(a, b)  (((a) > (b)) ? b : a)
//! Return the minimum of a and b (float).
#  define fminf(a, b) fmin(a, b)
//! Return the logarithm base 2 (double).
#  define log2(a) log(a)/log(2.0)
//! Return the logarithm base 2 (float).
#  define log2f(a) logf(a)/logf(2.0f)
#endif

//! cos() wrapper that uses the configured floating point precision.
#define JANUS_COS(a)     JANUS_REAL_SUFFIX(cos)(a)
//! sin() wrapper that uses the configured floating point precision.
#define JANUS_SIN(a)     JANUS_REAL_SUFFIX(sin)(a)
//! exp() wrapper that uses the configured floating point precision.
#define JANUS_EXP(a)     JANUS_REAL_SUFFIX(exp)(a)
//! fmod() wrapper that uses the configured floating point precision.
#define JANUS_FMOD(a, b) JANUS_REAL_SUFFIX(fmod)(a, b)
//! fmod() wrapper that uses the configured floating point precision.
#define JANUS_MODF(a, b) JANUS_REAL_SUFFIX(modf)(a, b)
//! fabs() wrapper that uses the configured floating point precision.
#define JANUS_FABS(a)    JANUS_REAL_SUFFIX(fabs)(a)
//! fmax() wrapper that uses the configured floating point precision.
#define JANUS_FMAX(a, b) JANUS_REAL_SUFFIX(fmax)(a, b)
//! fmax() wrapper that uses the configured floating point precision.
#define JANUS_FMIM(a, b) JANUS_REAL_SUFFIX(fmin)(a, b)
//! round() wrapper that uses the configured floating point precision.
#define JANUS_ROUND(a)   JANUS_REAL_SUFFIX(round)(a)
//! ceil() wrapper that uses the configured floating point precision.
#define JANUS_CEIL(a)    JANUS_REAL_SUFFIX(ceil)(a)
//! floor() wrapper that uses the configured floating point precision.
#define JANUS_FLOOR(a)   JANUS_REAL_SUFFIX(floor)(a)
//! trunc() wrapper that uses the configured floating point precision.
#define JANUS_TRUNC(a)   JANUS_REAL_SUFFIX(trunc)(a)
//! pow() wrapper that uses the configured floating point precision.
#define JANUS_POW(a, b)  JANUS_REAL_SUFFIX(pow)(a, b)
//! sqrt() wrapper that uses the configured floating point precision.
#define JANUS_SQRT(a)    JANUS_REAL_SUFFIX(sqrt)(a)
//! log2() wrapper that uses the configured floating point precision.
#define JANUS_LOG2(a)    JANUS_REAL_SUFFIX(log2)(a)
//! log() wrapper that uses the configured floating point precision.
#define JANUS_LOG(a)     JANUS_REAL_SUFFIX(log)(a)

//! cos() wrapper that uses the configured floating point precision.
#define JANUS_HP_COS(a)     JANUS_HIPRECISION_SUFFIX(cos)(a)
//! sin() wrapper that uses the configured floating point precision.
#define JANUS_HP_SIN(a)     JANUS_HIPRECISION_SUFFIX(sin)(a)

//! @}

#endif
