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

#ifndef JANUS_UTILS_MEMORY_H_INCLUDED_
#define JANUS_UTILS_MEMORY_H_INCLUDED_

// ISO C headers.
#include <stdlib.h>

#ifdef _MSC_VER
#  include <malloc.h>
#  ifdef JANUS_SYS_HAS___ALLOCA
#    ifndef alloca
#      define alloca _alloca
#    endif
#  endif
#else
#  ifdef JANUS_SYS_HAS___ALLOCA
#    include <alloca.h>
#  else
#    include <malloc.h>
#  endif
#endif

#define JANUS_UTILS_MEMORY_NEW(type, elms) \
  (type*)malloc(elms * sizeof(type))

#define JANUS_UTILS_MEMORY_NEW_ZERO(type, elms) \
  (type*)calloc(elms, sizeof(type))

#define JANUS_UTILS_MEMORY_REALLOC(ptr, type, elms) \
  (type*)realloc(ptr, elms * sizeof(type))

#define JANUS_UTILS_MEMORY_FREE(ptr) \
  if (ptr != 0) \
  { \
    free(ptr); \
    ptr = 0; \
  }

#if defined(JANUS_SYS_HAS___ALLOCA)

#define JANUS_UTILS_MEMORY_ALLOCA(type, elms) \
  (type*)alloca(elms * sizeof(type))

#define JANUS_UTILS_MEMORY_ALLOCA_FREE(ptr) \
  {}

#else

#define JANUS_UTILS_MEMORY_ALLOCA(type, elms) \
  (type*)malloc(elms * sizeof(type))

#define JANUS_UTILS_MEMORY_ALLOCA_FREE(ptr) \
  free(ptr)

#endif

#endif
