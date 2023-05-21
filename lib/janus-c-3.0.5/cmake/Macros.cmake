##########################################################################
# JANUS is a simple, robust, open standard signalling method for         #
# underwater communications. See <http://www.januswiki.org> for details. #
##########################################################################
# Example software implementations provided by STO CMRE are subject to   #
# Copyright (C) 2008-2018 STO Centre for Maritime Research and           #
# Experimentation (CMRE)                                                 #
#                                                                        #
# This is free software: you can redistribute it and/or modify it        #
# under the terms of the GNU General Public License version 3 as         #
# published by the Free Software Foundation.                             #
#                                                                        #
# This program is distributed in the hope that it will be useful, but    #
# WITHOUT ANY WARRANTY; without even the implied warranty of FITNESS     #
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for       #
# more details.                                                          #
#                                                                        #
# You should have received a copy of the GNU General Public License      #
# along with this program. If not, see <http://www.gnu.org/licenses/>.   #
##########################################################################
# Author: Ricardo Martins                                                #
##########################################################################

include(CheckSymbolExists)
include(CheckLibraryExists)
include(CheckIncludeFiles)
include(CheckFunctionExists)
include(CheckTypeSize)
include(CheckCSourceCompiles)
include(CheckCCompilerFlag)
include(CheckIncludeFile)
include(FindPkgConfig)

##########################################################################
#                            Compiler Tests                              #
##########################################################################
macro(janus_probe_cxx)
  message(STATUS "")
  message(STATUS "***************************************")
  message(STATUS "***         Probing Compiler        ***")
  message(STATUS "***************************************")

  # Intel C Compiler
  if(NOT JANUS_CXX_NAME)
    check_symbol_exists(__INTEL_COMPILER stdio.h JANUS_CXX_INTEL)
    if(JANUS_CXX_INTEL)
      set(JANUS_CXX_NAME "Intel")
      set(JANUS_CXX_CANONICAL "intel")
      if(CMAKE_BUILD_TYPE MATCHES "Debug")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
      endif(CMAKE_BUILD_TYPE MATCHES "Debug")
    endif(JANUS_CXX_INTEL)
  endif(NOT JANUS_CXX_NAME)

  # Sun Studio compiler.
  if(NOT JANUS_CXX_NAME)
    check_symbol_exists(__SUNPRO_C stdio.h JANUS_CXX_SUN)
    if(JANUS_CXX_SUN)
      set(JANUS_CXX_NAME "Sun Studio/Sun Pro")
      set(JANUS_CXX_CANONICAL "sunpro")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -library=stlport4")
    endif(JANUS_CXX_SUN)
  endif(NOT JANUS_CXX_NAME)

  # LLVM/Clang compiler.
  if(NOT JANUS_CXX_NAME)
    check_symbol_exists(__clang__ stdio.h JANUS_CXX_CLANG)
    if(JANUS_CXX_CLANG)
      set(JANUS_CXX_NAME "LLVM/Clang")
      set(JANUS_CXX_CANONICAL "clang")
    endif(JANUS_CXX_CLANG)
  endif(NOT JANUS_CXX_NAME)

  # GNU C Compiler
  if(NOT JANUS_CXX_NAME)
    check_symbol_exists(__GNUC__ stdio.h JANUS_CXX_GNU)
    if(JANUS_CXX_GNU)
      exec_program(${CMAKE_CXX_COMPILER} ARGS -dumpfullversion -dumpversion OUTPUT_VARIABLE verinfo)
      string(REPLACE "." ";" gxxver ${verinfo})
      list(LENGTH gxxver gxxverlen)
      list(GET gxxver 0 gxxmaj)
      if(gxxverlen GREATER 1)
        list(GET gxxver 1 gxxmin)
      else(gxxverlen GREATER 1)
        set(gxxmin 0)
      endif(gxxverlen GREATER 1)
        

      # For now all minor versions of GCC v4 have a compatible ABI.
      if("${gxxmaj}" MATCHES "4")
        set(gxxmin "x")
      endif("${gxxmaj}" MATCHES "4")

      set(JANUS_CXX_NAME "GCC v${gxxmaj}.${gxxmin}")
      set(JANUS_CXX_CANONICAL "gcc${gxxmaj}${gxxmin}")

      # This avoids spurious warnings from C++ STL code.
      if("${gxxmaj}" MATCHES "3")
        set(JANUS_CXX_FLAGS "${JANUS_CXX_FLAGS} -Wno-uninitialized")
      endif("${gxxmaj}" MATCHES "3")

      check_c_compiler_flag(-fexceptions has_fexceptions)
      if(has_fexceptions)
        set(JANUS_CXX_FLAGS "${JANUS_CXX_FLAGS} -fexceptions")
      endif(has_fexceptions)

      check_c_compiler_flag(-Wno-long-long has_wno_long_long)
      if(has_wno_long_long)
        set(JANUS_CXX_FLAGS "${JANUS_CXX_FLAGS} -Wno-long-long")
      endif(has_wno_long_long)

      if(PROFILE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pg")
      endif(PROFILE)

      if(CMAKE_SIZEOF_VOID_P EQUAL 8 OR NOT JANUS_NO_PIC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
      endif()

      set(JANUS_CXX_FLAGS_STRICT "-std=c99 -Wall -Wshadow -pedantic")
      set(JANUS_CXX_FLAGS_LOOSE  "")
      set(JANUS_CXX_FLAGS_DEBUG  "")
    endif(JANUS_CXX_GNU)
  endif(NOT JANUS_CXX_NAME)

  # Microsoft compiler
  if(NOT JANUS_CXX_NAME)
    check_symbol_exists(_MSC_VER stdio.h JANUS_CXX_MICROSOFT)
    if(JANUS_CXX_MICROSOFT)
      set(JANUS_CXX_NAME "Microsoft")

      if(MSVC60)
        message(FATAL_ERROR "Visual Studio 6.0 is not supported")
      elseif(MSVC12)
        set(msv_version "2012")
      elseif(MSVC10)
        set(msv_version "2010")
      elseif(MSVC90)
        set(msv_version "2008")
      elseif(MSVC80)
        set(msv_version "2005")
      elseif(MSVC71)
        set(msv_version "2003")
      elseif(MSVC70)
        set(msv_version "2002")
      endif(MSVC60)

      set(JANUS_CXX_CANONICAL "vs${msv_version}")
      set(JANUS_CXX_FLAGS "${JANUS_CXX_FLAGS} /wd4251 /wd4244 /wd4267")
      set(JANUS_CXX_FLAGS_STRICT "")
      set(JANUS_CXX_FLAGS_LOOSE  "")
      set(JANUS_CXX_FLAGS_DEBUG  "")

      add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_DEPRECATE)
    endif(JANUS_CXX_MICROSOFT)
  endif(NOT JANUS_CXX_NAME)

  # Unknown compiler
  if(NOT JANUS_CXX_NAME)
    set(JANUS_CXX_NAME "Unknown")
    set(JANUS_CXX_CANONICAL "unk")
    set(JANUS_CXX_UNKNOWN 1)
  endif(NOT JANUS_CXX_NAME)
endmacro(janus_probe_cxx)

##########################################################################
#                                CPU Tests                               #
##########################################################################
macro(janus_test_CPU cpu bits endianess macro header)
  string(TOUPPER ${macro} output)
  string(TOUPPER ${cpu} ucpu)
  string(TOLOWER ${cpu} lcpu)

  set(gbits ${bits})

  if(NOT JANUS_CPU)
    set(output "JANUS_SYS_HAS_${output}")

    check_c_source_compiles("#if !defined(${macro})\n#error undefined macro\n#endif\nint main(void){return 0;}" ${output})

    if(${output})
      check_c_source_compiles("#if defined(__arch64__)\n#else\n#error not defined\n#endif\nint main(void) {return 0; }\n" __ARCH64__)
      if(__ARCH64__)
        set(gbits 64)
      endif(__ARCH64__)

      set(JANUS_CPU 1)
      set(JANUS_CPU_${ucpu} 1)
      set(JANUS_CPU_${gbits}B 1)

      set(JANUS_CPU_NAME "${cpu}")
      set(JANUS_CPU_CANONICAL "${lcpu}")
      set(JANUS_CPU_UCASE "${ucpu}")
      set(JANUS_CPU_BITS "${gbits}")

      if(${endianess} STREQUAL "big")
        set(JANUS_CPU_BIG_ENDIAN 1)
        set(JANUS_CPU_ENDIANNESS "Big Endian")
      elseif(${endianess} STREQUAL "little")
        set(JANUS_CPU_LITTLE_ENDIAN 1)
        set(JANUS_CPU_ENDIANNESS "Little Endian")
      endif(${endianess} STREQUAL "big")
    endif(${output})
  endif(NOT JANUS_CPU)
endmacro(janus_test_CPU cpu bits macro header)

macro(janus_probe_cpu)
  message(STATUS "")
  message(STATUS "***************************************")
  message(STATUS "***       Probing Target CPU        ***")
  message(STATUS "***************************************")

  janus_test_cpu(x86      64     little   __amd64__       cstdio)
  janus_test_cpu(x86      64     little   _AMD64_         windows.h)
  janus_test_cpu(x86      64     little   __x86_64__      cstdio)
  janus_test_cpu(x86      64     little   _M_X64          windows.h)
  janus_test_cpu(x86      32     little   __i386__        cstdio)
  janus_test_cpu(x86      32     little   _M_IX86         windows.h)
  janus_test_cpu(x86      32     little   __x86_32__      cstdio)
  janus_test_cpu(PowerPC  64     big      __powerpc64__   cstdio)
  janus_test_cpu(PowerPC  32     big      __powerpc__     cstdio)
  janus_test_cpu(PowerPC  32     big      __ppc__         cstdio)
  janus_test_cpu(PowerPC  32     big      PPC             cstdio)
  janus_test_cpu(Alpha    32     big      __alpha__       cstdio)
  janus_test_cpu(ARMv4    32     unknown  __ARM_ARCH_4T__ cstdio)
  janus_test_cpu(ARMv5    32     unknown  __ARM_ARCH_5T__ cstdio)
  janus_test_cpu(ARMv7    32     unknown  __ARM_ARCH_7A__ cstdio)
  janus_test_cpu(ARM      32     unknown  __arm__         cstdio)
  janus_test_cpu(SPARC    32     big      __sparc__       cstdio)
  janus_test_cpu(SPARCv9  64     big      __sparcv9       cstdio)
  janus_test_cpu(SPARCv8  32     big      __sparcv8       cstdio)
  janus_test_cpu(MIPSEL   32     little   __MIPSEL__      cstdio)
  janus_test_cpu(MIPSEB   32     big      __MIPSEB__      cstdio)
  janus_test_cpu(AVR32    32     big      __avr32__       cstdio)

  # Unknown architecture
  if(NOT JANUS_CPU)
    set(JANUS_CPU_NAME "Unknown")
    set(JANUS_CPU_UNKNOWN 1)
  endif(NOT JANUS_CPU)

  # If we still don't know the architecture endianness we try macro tests.
  if(NOT JANUS_CPU_LITTLE_ENDIAN AND NOT JANUS_CPU_BIG_ENDIAN)
    check_c_source_compiles("
#include <sys/types.h>
#include <sys/param.h>

#ifdef __BYTE_ORDER
#  if (__BYTE_ORDER == __BIG_ENDIAN) || defined(_BIG_ENDIAN)
int main(void) { return 0; }
#  endif
#endif
" JANUS_CPU_BIG_ENDIAN)

    check_c_source_compiles("
#include <sys/types.h>
#include <sys/param.h>

#ifdef __BYTE_ORDER
#  if __BYTE_ORDER == __LITTLE_ENDIAN
int main(void) { return 0; }
#  endif
#endif
" JANUS_CPU_LITTLE_ENDIAN)

    check_c_source_compiles("#if !defined(__ARMEL__)\n#error no __ARMEL__\n#endif\nint main(void) {return 0;}\n" armel)
    if(armel)
      set(JANUS_CPU_LITTLE_ENDIAN 1)
    endif(armel)

    check_c_source_compiles("#if !defined(__ARMEB__)\n#error no __ARMEB__\n#endif\nint main(void) {return 0;}\n" armeb)
    if(armeb)
      set(JANUS_CPU_BIG_ENDIAN 1)
    endif(armeb)

  endif(NOT JANUS_CPU_LITTLE_ENDIAN AND NOT JANUS_CPU_BIG_ENDIAN)

  if(JANUS_CPU_LITTLE_ENDIAN)
    set(JANUS_CPU_ENDIANNESS "Little Endian")
  elseif(JANUS_CPU_BIG_ENDIAN)
    set(JANUS_CPU_ENDIANNESS "Big Endian")
  else(JANUS_CPU_BIG_ENDIAN)
    message(FATAL_ERROR "Unable to find CPU endianness.")
  endif(JANUS_CPU_LITTLE_ENDIAN)

  # ARM Floating Point Unit
  if(JANUS_CXX_GNU)
    check_c_source_compiles("
#if !defined(__arm__)
#  error not ARM architecture
#endif

#if defined(__VFP_FP__)
#  error normal doubles.
#endif

int main(void) { return 0; }
" JANUS_CPU_MIXED_ENDIAN_DOUBLES)
    if(JANUS_CPU_MIXED_ENDIAN_DOUBLES)
      set(JANUS_CPU_ENDIANESS "${JANUS_CPU_ENDIANNESS} / Mixed Endian Doubles")
    endif(JANUS_CPU_MIXED_ENDIAN_DOUBLES)
  endif(JANUS_CXX_GNU)
endmacro(janus_probe_cpu)

##########################################################################
#                             System Tests                               #
##########################################################################
macro(janus_probe_os)
  message(STATUS "")
  message(STATUS "***************************************")
  message(STATUS "***    Probing Operating System     ***")
  message(STATUS "***************************************")

  # GNU/Linux 2.4
  if(NOT JANUS_OS_NAME)
    check_c_source_compiles("
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 5, 0)
#  error Linux 2.5 or superior
#endif
" OS_LINUX24)
    if(OS_LINUX24)
      set(JANUS_OS_NAME "Linux2.4")
      set(JANUS_OS_CANONICAL "linux24")
      set(JANUS_OS_LINUX 1)
      set(JANUS_OS_LINUX24 1)
      set(JANUS_OS_POSIX 1)
    endif(OS_LINUX24)
  endif(NOT JANUS_OS_NAME)

  # GNU/Linux 2.6
  if(NOT JANUS_OS_NAME)
    check_symbol_exists(__linux__ stdio.h OS_LINUX)
    if(OS_LINUX)
      set(JANUS_OS_NAME "Linux")
      set(JANUS_OS_CANONICAL "linux")
      set(JANUS_OS_LINUX 1)
      set(JANUS_OS_POSIX 1)
    endif(OS_LINUX)
  endif(NOT JANUS_OS_NAME)

  # RTEMS
  if(NOT JANUS_OS_NAME)
    check_symbol_exists(__rtems__ stdio.h OS_RTEMS)
    if(OS_RTEMS)
      set(JANUS_OS_NAME "RTEMS")
      set(JANUS_OS_CANONICAL "rtems")
      set(JANUS_OS_RTEMS 1)
      set(JANUS_OS_POSIX 1)
    endif(OS_RTEMS)
  endif(NOT JANUS_OS_NAME)

  # eCos
  if(NOT JANUS_OS_NAME)
    check_symbol_exists(__ECOS__ stdio.h OS_ECOS)
    if(OS_ECOS)
      set(JANUS_OS_NAME "eCos")
      set(JANUS_OS_CANONICAL "ecos")
      set(JANUS_OS_ECOS 1)
      set(JANUS_OS_POSIX 1)
    endif(OS_ECOS)
  endif(NOT JANUS_OS_NAME)

  # Microsoft Windows (32 bits)
  if(NOT JANUS_OS_NAME)
    check_symbol_exists(_WIN32 stdio.h OS_WIN32)
    if(OS_WIN32)
      set(JANUS_OS_NAME "Windows")
      set(JANUS_OS_CANONICAL "windows")
      set(JANUS_OS_WINDOWS 1)
      add_definitions(-D_WIN32_WINNT=0x0501)
      set(CMAKE_REQUIRED_DEFINITIONS "${CMAKE_REQUIRED_DEFINITIONS} -D_WIN32_WINNT=0x0501")

      set(JANUS_PTHREADS_WIN32_FILES
        external/libraries/pthreads-win32/attr.c
        external/libraries/pthreads-win32/barrier.c
        external/libraries/pthreads-win32/cancel.c
        external/libraries/pthreads-win32/cleanup.c
        external/libraries/pthreads-win32/condvar.c
        external/libraries/pthreads-win32/create.c
        external/libraries/pthreads-win32/dll.c
        external/libraries/pthreads-win32/errno.c
        external/libraries/pthreads-win32/exit.c
        external/libraries/pthreads-win32/fork.c
        external/libraries/pthreads-win32/global.c
        external/libraries/pthreads-win32/misc.c
        external/libraries/pthreads-win32/mutex.c
        external/libraries/pthreads-win32/nonportable.c
        external/libraries/pthreads-win32/private.c
        external/libraries/pthreads-win32/rwlock.c
        external/libraries/pthreads-win32/sched.c
        external/libraries/pthreads-win32/semaphore.c
        external/libraries/pthreads-win32/signal.c
        external/libraries/pthreads-win32/spin.c
        external/libraries/pthreads-win32/sync.c
        external/libraries/pthreads-win32/tsd.c)

      set(CMAKE_REQUIRED_INCLUDES ${PROJECT_SOURCE_DIR}/external/libraries/pthreads-win32)
      include_directories(${PROJECT_SOURCE_DIR}/external/libraries/pthreads-win32)

      set_source_files_properties(${JANUS_PTHREADS_WIN32_FILES}
        PROPERTIES COMPILE_FLAGS "-DPTW32_BUILD -DHAVE_CONFIG_H")

      list(APPEND JANUS_EXTERNAL_FILES ${JANUS_PTHREADS_WIN32_FILES})

      set(JANUS_SYS_HAS_PTHREAD_H 1)
      set(JANUS_SYS_HAS_PTHREAD 1)
      set(JANUS_SYS_HAS_PTHREAD_MUTEX 1)
      set(JANUS_SYS_HAS_PTHREAD_COND 1)
      set(JANUS_SYS_HAS_PTHREAD_BARRIER 1)
      set(JANUS_SYS_HAS_PTHREAD_RWLOCK 1)
      set(JANUS_SYS_HAS_PTHREAD_KEY 1)
      set(JANUS_SYS_HAS_STRUCT_TIMESPEC 1)
    endif(OS_WIN32)
  endif(NOT JANUS_OS_NAME)

  # MacOS X / Darwin
  if(NOT JANUS_OS_NAME)
    check_symbol_exists(__APPLE__ stdio.h OS_DARWIN)
    if(OS_DARWIN)
      set(JANUS_OS_NAME "Darwin")
      set(JANUS_OS_CANONICAL "darwin")
      set(JANUS_OS_DARWIN 1)
      set(JANUS_OS_POSIX 1)
    endif(OS_DARWIN)
  endif(NOT JANUS_OS_NAME)

  # FreeBSD
  if(NOT JANUS_OS_NAME)
    check_symbol_exists(__DragonFly__ stdio.h OS_DRAGONFLY)
    if(OS_DRAGONFLY)
      set(JANUS_OS_NAME "DragonFly")
      set(JANUS_OS_CANONICAL "dragonfly")
      set(JANUS_OS_FREEBSD 1)
      set(JANUS_OS_POSIX 1)
      set(JANUS_OS_BSD 1)
    endif(OS_DRAGONFLY)
  endif(NOT JANUS_OS_NAME)

  if(NOT JANUS_OS_NAME)
    check_symbol_exists(__FreeBSD__ stdio.h OS_FREEBSD)
    if(OS_FREEBSD)
      set(JANUS_OS_NAME "FreeBSD")
      set(JANUS_OS_CANONICAL "freebsd")
      set(JANUS_OS_FREEBSD 1)
      set(JANUS_OS_POSIX 1)
      set(JANUS_OS_BSD 1)
    endif(OS_FREEBSD)
  endif(NOT JANUS_OS_NAME)

  if(NOT JANUS_OS_NAME)
    check_symbol_exists(__OpenBSD__ stdio.h OS_OPENBSD)
    if(OS_OPENBSD)
      set(JANUS_OS_NAME "OpenBSD")
      set(JANUS_OS_CANONICAL "openbsd")
      set(JANUS_OS_OPENBSD 1)
      set(JANUS_OS_POSIX 1)
      set(JANUS_OS_BSD 1)
    endif(OS_OPENBSD)
  endif(NOT JANUS_OS_NAME)

  if(NOT JANUS_OS_NAME)
    check_symbol_exists(__NetBSD__ stdio.h OS_NETBSD)
    if(OS_NETBSD)
      set(JANUS_OS_NAME "NetBSD")
      set(JANUS_OS_CANONICAL "netbsd")
      set(JANUS_OS_NETBSD 1)
      set(JANUS_OS_POSIX 1)
      set(JANUS_OS_BSD 1)
    endif(OS_NETBSD)
  endif(NOT JANUS_OS_NAME)

  # SUN Solaris
  if(NOT JANUS_OS_NAME)
    check_symbol_exists(__sun stdio.h OS_SUNOS)
    if(OS_SUNOS)
      set(JANUS_OS_NAME "Solaris")
      set(JANUS_OS_CANONICAL "solaris")
      set(JANUS_OS_SOLARIS 1)
      set(JANUS_OS_POSIX 1)
    endif(OS_SUNOS)
  endif(NOT JANUS_OS_NAME)

  # QNX Neutrino
  if(NOT JANUS_OS_NAME)
    check_symbol_exists(__QNXNTO__ stdio.h OS_QNX6)
    if(OS_QNX6)
      set(JANUS_OS_NAME "QNX6")
      set(JANUS_OS_CANONICAL "qnx6")
      set(JANUS_OS_QNX6 1)
      set(JANUS_OS_POSIX 1)
    endif(OS_QNX6)
  endif(NOT JANUS_OS_NAME)

  # Unknown System
  if(NOT JANUS_OS_NAME)
    set(JANUS_OS_NAME "Unknown")
    set(JANUS_OS_CANONICAL "unknown")
  endif(NOT JANUS_OS_NAME)
endmacro(janus_probe_os)

##########################################################################
#                          C and C++ Library Tests                       #
##########################################################################
macro(janus_probe_cxx_lib)
  message(STATUS "")
  message(STATUS "***************************************")
  message(STATUS "***      Probing C/C++ Library      ***")
  message(STATUS "***************************************")

  # LLVM libc++
  if(NOT JANUS_CLIB_NAME)
    check_c_source_compiles("#include <cstddef>\n int main(void) { return _LIBCPP_VERSION; }" LLVM_LIBC)
    if(LLVM_LIBC)
      set(JANUS_CLIB_NAME "libc++")
      set(JANUS_CLIB_CANONICAL "libcpp")
      set(JANUS_CLIB_LIBCPP 1)
    endif(LLVM_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  # uClibc
  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(__UCLIBC__ stdio.h UC_LIBC)
    if(UC_LIBC)
      set(JANUS_CLIB_NAME "uClibc")
      set(JANUS_CLIB_CANONICAL "uclibc")
      set(JANUS_CLIB_UC 1)
    endif(UC_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  # GNU C library
  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(__GLIBC__ stdio.h GNU_LIBC)
    if(GNU_LIBC)
      set(JANUS_CLIB_NAME "GNU")
      set(JANUS_CLIB_CANONICAL "glibc")
      set(JANUS_CLIB_GNU 1)
    endif(GNU_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  # Newlib
  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(_NEWLIB_VERSION stdio.h NEW_LIBC)
    if(NEW_LIBC)
      set(JANUS_CLIB_NAME "Newlib")
      set(JANUS_CLIB_CANONICAL "newlib")
      set(JANUS_CLIB_NEWLIB 1)
    endif(NEW_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  # eCos
  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(__ECOS__ stdio.h ECOS_LIBC)
    if(ECOS_LIBC)
      set(JANUS_CLIB_NAME "eCos")
      set(JANUS_CLIB_CANONICAL "ecos")
      set(JANUS_CLIB_ECOS 1)
    endif(ECOS_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  # MinGW C library
  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(__MINGW32_VERSION stdio.h MINGW_LIBC)
    if(MINGW_LIBC)
      set(JANUS_CLIB_NAME "MinGW")
      set(JANUS_CLIB_CANONICAL "mingw")
      set(JANUS_CLIB_MINGW 1)
    endif(MINGW_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  # Microsoft C library
  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(_MSC_VER stdio.h MICROSOFT_LIBC)
    if(MICROSOFT_LIBC)
      set(JANUS_CLIB_NAME "Microsoft")
      set(JANUS_CLIB_CANONICAL "microsoft")
      set(JANUS_CLIB_MICROSOFT 1)
    endif(MICROSOFT_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  # MacOS X / Darwin library
  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(__APPLE__ stdio.h APPLE_LIBC)
    if(APPLE_LIBC)
      set(JANUS_CLIB_NAME "Apple")
      set(JANUS_CLIB_CANONICAL "apple")
      set(JANUS_CLIB_APPLE 1)
    endif(APPLE_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  # SUN Library
  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(__sun stdio.h SUN_LIBC)
    if(SUN_LIBC)
      set(JANUS_CLIB_NAME "SUN")
      set(JANUS_CLIB_CANONICAL "sun")
      set(JANUS_CLIB_SUN 1)
    endif(SUN_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  # Dinkum Library
  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(_HAS_DINKUM_CLIB stdio.h DINKUM_LIBC)
    if(DINKUM_LIBC)
      set(JANUS_CLIB_NAME "Dinkum")
      set(JANUS_CLIB_CANONICAL "dinkum")
      set(JANUS_CLIB_DINKUM 1)
    endif(DINKUM_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  # BSD Library
  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(__DragonFly__ stdio.h DRAGONFLY_LIBC)
    if(DRAGONFLY_LIBC)
      set(JANUS_CLIB_NAME "BSD")
      set(JANUS_CLIB_CANONICAL "bsd")
      set(JANUS_CLIB_BSD 1)
    endif(DRAGONFLY_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(__FreeBSD__ stdio.h FREEBSD_LIBC)
    if(FREEBSD_LIBC)
      set(JANUS_CLIB_NAME "BSD")
      set(JANUS_CLIB_CANONICAL "bsd")
      set(JANUS_CLIB_BSD 1)
    endif(FREEBSD_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(__OpenBSD__ stdio.h OPENBSD_LIBC)
    if(OPENBSD_LIBC)
      set(JANUS_CLIB_NAME "BSD")
      set(JANUS_CLIB_CANONICAL "bsd")
      set(JANUS_CLIB_BSD 1)
    endif(OPENBSD_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  if(NOT JANUS_CLIB_NAME)
    check_symbol_exists(__NetBSD__ stdio.h NETBSD_LIBC)
    if(NETBSD_LIBC)
      set(JANUS_CLIB_NAME "BSD")
      set(JANUS_CLIB_CANONICAL "bsd")
      set(JANUS_CLIB_BSD 1)
    endif(NETBSD_LIBC)
  endif(NOT JANUS_CLIB_NAME)

  # Unknown C library
  if(NOT JANUS_CLIB_NAME)
    set(JANUS_CLIB_NAME "Unknown")
    set(JANUS_CLIB_CANONICAL "unk")
    set(JANUS_CLIB_UNKNOWN 1)
  endif(NOT JANUS_CLIB_NAME)
endmacro(janus_probe_cxx_lib)

##########################################################################
#                          Assorted Libraries Tests                      #
##########################################################################
# This macro checks if the symbol exists in the library and if it
# does, it appends library to the list.
macro(janus_test_lib library symbol)
  string(REPLACE "." "_" output ${library})
  string(REPLACE "/" "_" output ${output})
  string(REPLACE "-" "_" output ${output})
  string(TOUPPER JANUS_SYS_HAS_LIB_${output} output)

  CHECK_LIBRARY_EXISTS(${library} ${symbol} "" ${output})

  if(${output})
    set(JANUS_SYS_LIBS ${JANUS_SYS_LIBS} ${library})
    set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES} ${library})
  endif(${output})
endmacro(janus_test_lib)

macro(janus_probe_libs)
  message(STATUS "")
  message(STATUS "***************************************")
  message(STATUS "***    Probing System Libraries     ***")
  message(STATUS "***************************************")

  janus_test_lib(rt clock_gettime)
  janus_test_lib(ws2_32 getch)
  janus_test_lib(wsock32 getch)
  janus_test_lib(iphlpapi getch)
  janus_test_lib(socket setsockopt)
  janus_test_lib(pthread pthread_self)
  janus_test_lib(m pow)
  janus_test_lib(xnet inet_addr)
  janus_test_lib(nsl inet_ntop)
  janus_test_lib(dl dlopen)

  # find_package(Qt4)
  # set(QT_USE_QTXML 1)
  # set(QT_USE_QTUITOOLS 1)
  # set(QT_USE_QTNETWORK 1)
  # include(${QT_USE_FILE})

endmacro(janus_probe_libs)

##########################################################################
#                             Headers Tests                              #
##########################################################################
macro(janus_test_header header)
  string(REPLACE "." "_" output ${header})
  string(REPLACE "/" "_" output ${output})
  string(REPLACE "-" "_" output ${output})
  string(TOUPPER ${output} output)

  set(output "JANUS_SYS_HAS_${output}")

  check_include_file(${header} ${output})

  if(${output})
    set(JANUS_SYS_HEADERS "${JANUS_SYS_HEADERS}#define ${output}\n")
  else(${output})
    set(JANUS_SYS_HEADERS "${JANUS_SYS_HEADERS}//#undef ${output}\n")
  endif(${output})
endmacro(janus_test_header header)

macro(janus_test_header_deps header deps)
  set(program "")
  string(REPLACE "." "_" output ${header})
  string(REPLACE "/" "_" output ${output})
  string(REPLACE "-" "_" output ${output})
  string(TOUPPER ${output} output)

  set(output "JANUS_SYS_HAS_${output}")

  # Dependency headers.
  foreach(dep ${deps})
    string(REPLACE "." "_" dep_var ${dep})
    string(REPLACE "/" "_" dep_var ${dep_var})
    string(REPLACE "-" "_" dep_var ${dep_var})
    string(TOUPPER ${dep_var} dep_var)
    set(dep_var "JANUS_SYS_HAS_${dep_var}")

    if(${dep_var})
      set(program "${program}#include <${dep}>\n")
    endif(${dep_var})
  endforeach(dep)

  set(program "${program}\n#include <${header}>\nint main(void){ return 0; }\n")

  # Compile.
  if(NOT ${output})
    check_c_source_compiles("${program}" ${output})
  endif(NOT ${output})

  if(${output})
    set(JANUS_SYS_HEADERS "${JANUS_SYS_HEADERS}#define ${output}\n")
  else(${output})
    set(JANUS_SYS_HEADERS "${JANUS_SYS_HEADERS}//#undef ${output}\n")
  endif(${output})
endmacro(janus_test_header_deps header deps)

macro(janus_probe_headers)
  message(STATUS "")
  message(STATUS "***************************************")
  message(STATUS "***     Probing System Headers      ***")
  message(STATUS "***************************************")

  janus_test_header(stdio.h)
  janus_test_header(time.h)
  janus_test_header(arpa/inet.h)
  janus_test_header(dirent.h)
  janus_test_header(dlfcn.h)
  janus_test_header(fcntl.h)
  janus_test_header(inttypes.h)
  janus_test_header(netdb.h)
  janus_test_header(pthread.h)
  janus_test_header(signal.h)
  janus_test_header(stdint.h)
  janus_test_header(sys/io.h)
  janus_test_header(sys/ioctl.h)
  janus_test_header(sys/procfs.h)
  janus_test_header(sys/signal.h)
  janus_test_header(sys/stat.h)
  janus_test_header(sys/statfs.h)
  janus_test_header(sys/sendfile.h)
  janus_test_header(sys/time.h)
  janus_test_header(sys/types.h)
  janus_test_header(sys/file.h)
  janus_test_header(sys/wait.h)
  janus_test_header(sys/vfs.h)
  janus_test_header(sys/statvfs.h)
  janus_test_header(sys/syscall.h)
  janus_test_header(termios.h)
  janus_test_header(unistd.h)
  janus_test_header(windows.h)
  janus_test_header(direct.h)
  janus_test_header(winsock2.h)
  janus_test_header(ws2tcpip.h)
  janus_test_header(wspiapi.h)
  janus_test_header(ws2spi.h)
  janus_test_header(mach-o/dyld.h)
  janus_test_header(process.h)
  janus_test_header(bsp.h)
  janus_test_header(sys/param.h)
  janus_test_header(sys/mount.h)
  janus_test_header(linux/videodev2.h)
  janus_test_header(sched.h)
  janus_test_header(poll.h)
  janus_test_header(ifaddrs.h)
  janus_test_header(semaphore.h)
  janus_test_header(alloca.h)

  # A few systems/libraries have non self contained headers (notably
  # OpenBSD and RTEMS), to overcome this we perform the following
  # header tests listing headers that must be included first.
  janus_test_header_deps(sys/socket.h "sys/types.h")
  janus_test_header_deps(netinet/in.h "sys/types.h")
  janus_test_header_deps(sys/select.h "sys/types.h")
  janus_test_header_deps(sys/sysctl.h "sys/types.h")
  janus_test_header_deps(sys/resource.h "sys/types.h")
  janus_test_header_deps(sys/mman.h "sys/types.h")
  janus_test_header_deps(net/if.h "sys/types.h;sys/socket.h")
  janus_test_header_deps(timepps.h "unistd.h")
  janus_test_header_deps(iphlpapi.h "windows.h")
endmacro(janus_probe_headers)

##########################################################################
#                             Functions Tests                            #
##########################################################################
macro(janus_test_function function return types headers output)
  set(program "")
  set(arguments "")

  # Include headers.
  foreach(header ${headers})

    string(REPLACE "." "_" header_var ${header})
    string(REPLACE "/" "_" header_var ${header_var})
    string(REPLACE "-" "_" header_var ${header_var})
    string(TOUPPER ${header_var} header_var)
    set(header_var "JANUS_SYS_HAS_${header_var}")

    if(${header_var})
      set(program "${program}#include <${header}>\n")
    endif(${header_var})
  endforeach(header)

  # Begin main.
  set(program "${program}\nint main(void) {\n")

  set(variable "x")
  foreach(type ${types})

    # Treat function pointers
    if(type MATCHES "\\(\\*\\)")
      string(REPLACE "(*)" "(*type_${variable})" typedef ${type})
      set(program "${program}\ntypedef ${typedef};\n")
      set(type "type_${variable}")
    endif(type MATCHES "\\(\\*\\)")

    # Declare function arguments.
    if(type MATCHES "\\*\\*$")
      set(pointer "&")
      string(REPLACE "**" "*" type ${type})
    elseif(type MATCHES "\\*$")
      set(pointer "&")
      string(REPLACE "*" "" type ${type})
    else(type MATCHES "\\*\\*$")
      set(pointer "")
    endif(type MATCHES "\\*\\*$")

    # Treat void*
    if(type MATCHES "void")
      set(type "char")
    endif(type MATCHES "void")

    set(program "${program}${type} ${variable};\n")

    # Construct argument list
    if(NOT variable STREQUAL "x")
      set(arguments "${arguments}, ")
    endif(NOT variable STREQUAL "x")
    set(arguments "${arguments}${pointer}${variable}")

    set(variable "${variable}x")
  endforeach(type)

  # End main.
  if(${return} STREQUAL "void")
    set(program "${program}\n${function}(${arguments});\nreturn 0;\n}\n")
  else(${return} STREQUAL "void")
    set(program "${program}\n ${return} rv = ${function}(${arguments});\nreturn 0;\n}\n")
  endif(${return} STREQUAL "void")

  # Compile.
  if(NOT ${output})
    set(CMAKE_REQUIRED_FLAGS ${JANUS_CXX_FLAGS_STRICT})
    check_c_source_compiles("${program}" ${output})
  endif(NOT ${output})

  if(${output})
    set(JANUS_SYS_FUNCTIONS "${JANUS_SYS_FUNCTIONS}#define ${output}\n")
  else(${output})
    set(JANUS_SYS_FUNCTIONS "${JANUS_SYS_FUNCTIONS}//#undef ${output}\n")
  endif(${output})
endmacro(janus_test_function)

macro(janus_probe_functions)
  message(STATUS "")
  message(STATUS "***************************************")
  message(STATUS "***     Probing System Functions    ***")
  message(STATUS "***************************************")

  janus_test_function(strncpy_s
    "errno_t"
    "char*;size_t;char*;size_t"
    "windows.h"
    JANUS_SYS_HAS_STRNCPY_S)

  janus_test_function(strncpy
    "char*"
    "char*;char*;size_t"
    "cstring"
    JANUS_SYS_HAS_STRNCPY)

  janus_test_function(vsnprintf
    "int"
    "char*;size_t;char*;va_list"
    "cstdio;cstdarg"
    JANUS_SYS_HAS_VSNPRINTF)

  janus_test_function(vsnprintf_s
    "int"
    "char*;size_t;size_t;char*; va_list"
    "windows.h"
    JANUS_SYS_HAS_VSNPRINTF_S)

  janus_test_function(close
    "int"
    "int"
    "unistd.h"
    JANUS_SYS_HAS_CLOSE)

  janus_test_function(closesocket
    "int"
    "SOCKET"
    "winsock2.h"
    JANUS_SYS_HAS_CLOSESOCKET)

  janus_test_function(pthread_create
    "int"
    "pthread_t*;pthread_attr_t*;void *(*)(void*);void*"
    "pthread.h"
    JANUS_SYS_HAS_PTHREAD)

  janus_test_function(pthread_key_delete
    "int"
    "pthread_key_t"
    "pthread.h"
    JANUS_SYS_HAS_PTHREAD_KEY_DELETE)

  janus_test_function(pthread_sigmask
    "int"
    "int;sigset_t*;sigset_t*"
    "signal.h"
    JANUS_SYS_HAS_PTHREAD_SIGMASK)

  janus_test_function(pthread_kill
    "int"
    "pthread_t;int"
    "signal.h"
    JANUS_SYS_HAS_PTHREAD_KILL)

  janus_test_function(pthread_barrier_init
    "int"
    "pthread_barrier_t*;pthread_barrierattr_t*;unsigned"
    "pthread.h"
    JANUS_SYS_HAS_PTHREAD_BARRIER)

  janus_test_function(pthread_cond_init
    "int"
    "pthread_cond_t*;pthread_condattr_t*"
    "pthread.h"
    JANUS_SYS_HAS_PTHREAD_COND)

  janus_test_function(pthread_rwlock_init
    "int"
    "pthread_rwlock_t*;pthread_rwlockattr_t*"
    "pthread.h"
    JANUS_SYS_HAS_PTHREAD_RWLOCK)

  janus_test_function(pthread_mutex_init
    "int"
    "pthread_mutex_t*;pthread_mutexattr_t*"
    "pthread.h"
    JANUS_SYS_HAS_PTHREAD_MUTEX)

  janus_test_function(pthread_key_create
    "int"
    "pthread_key_t*;void (*)(void *)"
    "pthread.h"
    JANUS_SYS_HAS_PTHREAD_KEY)

  janus_test_function(pthread_condattr_setclock
    "int"
    "pthread_condattr_t*;clockid_t"
    "pthread.h"
    JANUS_SYS_HAS_PTHREAD_CONDATTR_SETCLOCK)

  janus_test_function(sigaction
    "int"
    "int;struct sigaction*;struct sigaction*"
    "signal.h"
    JANUS_SYS_HAS_SIGACTION)

  janus_test_function(mmap
    "void*"
    "void*;size_t;int;int;int;off_t"
    "sys/mman.h;sys/types.h"
    JANUS_SYS_HAS_MMAP)

  janus_test_function(mmap64
    "void*"
    "void*;size_t;int;int;int;off_t"
    "sys/mman.h;sys/types.h"
    JANUS_SYS_HAS_MMAP64)

  janus_test_function(round
    "double"
    "double"
    "cmath"
    JANUS_SYS_HAS_ROUND)

  janus_test_function(lround
    "long int"
    "double"
    "cmath"
    JANUS_SYS_HAS_LROUND)

  janus_test_function(getcwd
    "char*"
    "char*;size_t"
    "unistd.h"
    JANUS_SYS_HAS_GETCWD)

  janus_test_function(_getcwd
    "char*"
    "char*;int"
    "direct.h"
    JANUS_SYS_HAS__GETCWD)

  janus_test_function(FormatMessage
    "DWORD"
    "DWORD;void*;DWORD;DWORD;char*;DWORD;va_list*"
    "windows.h"
    JANUS_SYS_HAS_FORMAT_MESSAGE)

  janus_test_function(ioperm
    "int"
    "unsigned long;unsigned long;int"
    "unistd.h;sys/io.h"
    JANUS_SYS_HAS_IOPERM)

  janus_test_function(localtime
    "struct tm*"
    "time_t*"
    "time.h"
    JANUS_SYS_HAS_LOCALTIME)

  janus_test_function(localtime_r
    "struct tm*"
    "time_t*;struct tm*"
    "time.h"
    JANUS_SYS_HAS_LOCALTIME_R)

  janus_test_function(clock_nanosleep
    "int"
    "clockid_t;int;struct timespec*;struct timespec*"
    "time.h"
    JANUS_SYS_HAS_CLOCK_NANOSLEEP)

  janus_test_function(nanosleep
    "int"
    "struct timespec*;struct timespec*"
    "time.h"
    JANUS_SYS_HAS_NANOSLEEP)

  janus_test_function(strerror
    "char*"
    "int"
    "cstring"
    JANUS_SYS_HAS_STRERROR)

  janus_test_function(clock_gettime
    "int"
    "clockid_t;struct timespec*"
    "time.h"
    JANUS_SYS_HAS_CLOCK_GETTIME)

  janus_test_function(GetSystemTimeAsFileTime
    "void"
    "FILETIME*"
    "windows.h"
    JANUS_SYS_HAS_GET_SYSTEM_TIME_AS_FILE_TIME)

  janus_test_function(GetLastError
    "DWORD"
    ""
    "windows.h"
    JANUS_SYS_HAS_GET_LAST_ERROR)

  janus_test_function(CreateWaitableTimer
    "HANDLE"
    "SECURITY_ATTRIBUTES*;BOOL;char*"
    "windows.h"
    JANUS_SYS_HAS_CREATE_WAITABLE_TIMER)

  janus_test_function(GetModuleFileName
    "DWORD"
    "HMODULE;LPSTR;DWORD"
    "windows.h"
    JANUS_SYS_HAS_GET_MODULE_FILE_NAME)

  janus_test_function(LoadLibrary
    "HMODULE"
    "char*"
    "windows.h"
    JANUS_SYS_HAS_LOAD_LIBRARY)

  janus_test_function(TlsAlloc
    "DWORD"
    ""
    "windows.h"
    JANUS_SYS_HAS_TLS_ALLOC)

  janus_test_function(dlerror
    "const char*"
    ""
    "dlfcn.h"
    JANUS_SYS_HAS_DLERROR)

  janus_test_function(dlopen
    "void*"
    "char*;int"
    "dlfcn.h"
    JANUS_SYS_HAS_DLOPEN)

  janus_test_function(gettimeofday
    "int"
    "struct timeval*;struct timezone*"
    "sys/time.h"
    JANUS_SYS_HAS_GETTIMEOFDAY)

  janus_test_function(htonl
    "unsigned int"
    "unsigned int"
    "arpa/inet.h;windows.h;netinet/in.h"
    JANUS_SYS_HAS_HTONL)

  janus_test_function(htons
    "unsigned int"
    "unsigned int"
    "arpa/inet.h;windows.h;netinet/in.h"
    JANUS_SYS_HAS_HTONS)

  janus_test_function(ntohl
    "unsigned int"
    "unsigned int"
    "arpa/inet.h;windows.h;netinet/in.h"
    JANUS_SYS_HAS_NTOHL)

  janus_test_function(ntohs
    "unsigned int"
    "unsigned int"
    "arpa/inet.h;windows.h;netinet/in.h"
    JANUS_SYS_HAS_NTOHS)

  janus_test_function(inet_addr
    "unsigned long"
    "char*"
    "sys/types.h;sys/socket.h;arpa/inet.h;winsock2.h"
    JANUS_SYS_HAS_INET_ADDR)

  janus_test_function(inet_ntoa
    "char*"
    "struct in_addr"
    "sys/types.h;sys/socket.h;arpa/inet.h;winsock2.h"
    JANUS_SYS_HAS_INET_NTOA)

  janus_test_function(inet_aton
    "int"
    "char*;struct in_addr*"
    "netinet/in.h;sys/types.h;sys/socket.h;arpa/inet.h;winsock2.h"
    JANUS_SYS_HAS_INET_ATON)

  janus_test_function(inet_ntop
    "const char*"
    "int;void*;char*;socklen_t"
    "sys/types.h;sys/socket.h;arpa/inet.h;winsock2.h"
    JANUS_SYS_HAS_INET_NTOP)

  janus_test_function(inet_pton
    "int"
    "int;char*;void*"
    "sys/types.h;sys/socket.h;arpa/inet.h;winsock2.h"
    JANUS_SYS_HAS_INET_PTON)

  janus_test_function(getaddrinfo
    "int"
    "char*;char*;struct addrinfo*;struct addrinfo**"
    "sys/types.h;sys/socket.h;netdb.h;winsock2.h;ws2tcpip.h"
    JANUS_SYS_HAS_GETADDRINFO)

  janus_test_function(lstat
    "int"
    "char*;struct stat*"
    "sys/types.h;sys/stat.h;unistd.h"
    JANUS_SYS_HAS_LSTAT)

  janus_test_function(statfs64
    "int"
    "char*;struct statfs64*"
    "sys/param.h;sys/mount.h;sys/vfs.h"
    JANUS_SYS_HAS_STATFS64_DARWIN_LINUX)

  janus_test_function(mkdir
    "int"
    "char*;mode_t"
    "sys/stat.h;sys/types.h"
    JANUS_SYS_HAS_POSIX_MKDIR)

  janus_test_function(CreateDirectory
    "BOOL"
    "LPCTSTR;LPSECURITY_ATTRIBUTES"
    "windows.h"
    JANUS_SYS_HAS_CREATE_DIRECTORY)

  janus_test_function(select
    "int"
    "int;fd_set*;fd_set*;fd_set*;struct timeval*"
    "sys/types.h;sys/select.h;winsock2.h"
    JANUS_SYS_HAS_SELECT)

  janus_test_function(sendfile
    "ssize_t"
    "int;int;off_t*;size_t"
    "sys/sendfile.h"
    JANUS_SYS_HAS_LINUX_SENDFILE)

  janus_test_function(settimeofday
    "int"
    "struct timeval*;struct timezone*"
    "sys/time.h"
    JANUS_SYS_HAS_SETTIMEOFDAY)

  janus_test_function(socket
    "int"
    "int;int;int"
    "sys/types.h;sys/socket.h;winsock2.h"
    JANUS_SYS_HAS_SOCKET)

  janus_test_function(WSAStartup
    "int"
    "WORD;WSADATA*"
    "winsock2.h"
    JANUS_SYS_HAS_WSA_STARTUP)

  janus_test_function(WSACleanup
    "int"
    ""
    "winsock2.h"
    JANUS_SYS_HAS_WSA_CLEANUP)

  janus_test_function(WSAStringToAddress
    "int"
    "char*;int;WSAPROTOCOL_INFO*;SOCKADDR*;int*"
    "winsock2.h"
    JANUS_SYS_HAS_WSA_STRING_TO_ADDRESS)

  janus_test_function(WSAIoctl
    "int"
    "SOCKET;DWORD;void*;DWORD;void*;DWORD;LPDWORD;LPWSAOVERLAPPED;LPWSAOVERLAPPED_COMPLETION_ROUTINE"
    "winsock2.h"
    JANUS_SYS_HAS_WSA_IOCTL)

  janus_test_function(stat
    "int"
    "char*;struct stat*"
    "sys/types.h;sys/stat.h;unistd.h"
    JANUS_SYS_HAS_STAT)

  janus_test_function(strerror_r
    "char*"
    "int;char*;size_t"
    "cstring"
    JANUS_SYS_HAS_GNU_STRERROR_R)

  janus_test_function(strerror_r
    "int"
    "int;char*;size_t"
    "cstring"
    JANUS_SYS_HAS_POSIX_STRERROR_R)

  janus_test_function(CreateFile
    "HANDLE"
    "LPCSTR;DWORD;DWORD;SECURITY_ATTRIBUTES*;DWORD;DWORD;HANDLE"
    "windows.h"
    JANUS_SYS_HAS_CREATE_FILE)

  janus_test_function(ReadFile
    "BOOL"
    "HANDLE;LPVOID;DWORD;DWORD*;OVERLAPPED*"
    "windows.h"
    JANUS_SYS_HAS_READ_FILE)

  janus_test_function(WriteFile
    "BOOL"
    "HANDLE;LPCVOID;DWORD;DWORD*;OVERLAPPED*"
    "windows.h"
    JANUS_SYS_HAS_WRITE_FILE)

  janus_test_function(GetOverlappedResult
    "BOOL"
    "HANDLE;OVERLAPPED*;DWORD*;BOOL"
    "windows.h"
    JANUS_SYS_HAS_GET_OVERLAPPED_RESULT)

  janus_test_function(SetCommState
    "BOOL"
    "HANDLE;DCB*"
    "windows.h"
    JANUS_SYS_HAS_SET_COMM_STATE)

  janus_test_function(GetCommState
    "BOOL"
    "HANDLE;DCB*"
    "windows.h"
    JANUS_SYS_HAS_GET_COMM_STATE)

  janus_test_function(SetCommMask
    "BOOL"
    "HANDLE;DWORD"
    "windows.h"
    JANUS_SYS_HAS_SET_COMM_MASK)

  janus_test_function(PurgeComm
    "BOOL"
    "HANDLE;DWORD"
    "windows.h"
    JANUS_SYS_HAS_PURGE_COMM)

  janus_test_function(WaitCommEvent
    "BOOL"
    "HANDLE;DWORD*;OVERLAPPED*"
    "windows.h"
    JANUS_SYS_HAS_WAIT_COMM_EVENT)

  janus_test_function(SetCommBreak
    "BOOL"
    "HANDLE"
    "windows.h"
    JANUS_SYS_HAS_SET_COMM_BREAK)

  janus_test_function(ClearCommBreak
    "BOOL"
    "HANDLE"
    "windows.h"
    JANUS_SYS_HAS_CLEAR_COMM_BREAK)

  janus_test_function(ClearCommError
    "BOOL"
    "HANDLE;DWORD*;COMSTAT*"
    "windows.h"
    JANUS_SYS_HAS_CLEAR_COMM_ERROR)

  janus_test_function(WaitForSingleObject
    "DWORD"
    "HANDLE;DWORD"
    "windows.h"
    JANUS_SYS_HAS_WAIT_FOR_SINGLE_OBJECT)

  janus_test_function(CreateEvent
    "HANDLE"
    "SECURITY_ATTRIBUTES*;BOOL;BOOL;LPCSTR"
    "windows.h"
    JANUS_SYS_HAS_CREATE_EVENT)

  janus_test_function(SetCommState
    "BOOL"
    "HANDLE;DCB*"
    "windows.h"
    JANUS_SYS_HAS_SET_COMM_STATE)

  janus_test_function(GetCommState
    "BOOL"
    "HANDLE;DCB*"
    "windows.h"
    JANUS_SYS_HAS_GET_COMM_STATE)

  janus_test_function(SetCommMask
    "BOOL"
    "HANDLE;DWORD"
    "windows.h"
    JANUS_SYS_HAS_SET_COMM_MASK)

  janus_test_function(PurgeComm
    "BOOL"
    "HANDLE;DWORD"
    "windows.h"
    JANUS_SYS_HAS_PURGE_COMM)

  janus_test_function(SetCommBreak
    "BOOL"
    "HANDLE"
    "windows.h"
    JANUS_SYS_HAS_SET_COMM_BREAK)

  janus_test_function(ClearCommBreak
    "BOOL"
    "HANDLE"
    "windows.h"
    JANUS_SYS_HAS_CLEAR_COMM_BREAK)

  janus_test_function(random
    "long int"
    ""
    "cstdlib"
    JANUS_SYS_HAS_RANDOM)

  janus_test_function(srandom
    "void"
    "unsigned int"
    "cstdlib"
    JANUS_SYS_HAS_SRANDOM)

  janus_test_function(lrand48
    "long int"
    ""
    "cstdlib"
    JANUS_SYS_HAS_LRAND48)

  janus_test_function(srand48
    "void"
    "long int"
    "cstdlib"
    JANUS_SYS_HAS_SRAND48)

  janus_test_function(sched_yield
    "int"
    ""
    "sched.h"
    JANUS_SYS_HAS_SCHED_YIELD)

  janus_test_function(sched_get_priority_min
    "int"
    "int"
    "sched.h"
    JANUS_SYS_HAS_SCHED_GET_PRIORITY_MIN)

  janus_test_function(sched_get_priority_max
    "int"
    "int"
    "sched.h"
    JANUS_SYS_HAS_SCHED_GET_PRIORITY_MAX)

  janus_test_function(__sync_add_and_fetch
    "int"
    "int*;int"
    ""
    JANUS_SYS_HAS___SYNC_ADD_AND_FETCH)

  janus_test_function(__sync_sub_and_fetch
    "int"
    "int*;int"
    ""
    JANUS_SYS_HAS___SYNC_SUB_AND_FETCH)

  janus_test_function(fork
    "pid_t"
    ""
    "unistd.h"
    JANUS_SYS_HAS_FORK)

  janus_test_function(shm_unlink
    "int"
    "char*"
    "sys/mman.h;sys/stat.h;fcntl.h"
    JANUS_SYS_HAS_SHM_UNLINK)

  janus_test_function(shm_open
    "int"
    "char*;int;mode_t"
    "sys/mman.h;sys/stat.h;fcntl.h"
    JANUS_SYS_HAS_SHM_OPEN)

  janus_test_function(alloca
    "void*"
    "size_t"
    "alloca.h"
    JANUS_SYS_HAS_ALLOCA)

  janus_test_function(_alloca
    "void*"
    "size_t"
    "malloc.h"
    JANUS_SYS_HAS__ALLOCA)

  if(DEFINED JANUS_SYS_HAS_ALLOCA OR DEFINED JANUS_SYS_HAS__ALLOCA)
    # User can define JANUS_WITHOUT_ALLOCA=1 to avoid use of alloca.
    if(NOT DEFINED JANUS_WITHOUT_ALLOCA OR NOT ${JANUS_WITHOUT_ALLOCA})
      add_definitions(-DJANUS_SYS_HAS___ALLOCA=1)
    endif(NOT DEFINED JANUS_WITHOUT_ALLOCA OR NOT ${JANUS_WITHOUT_ALLOCA})
  endif(DEFINED JANUS_SYS_HAS_ALLOCA OR DEFINED JANUS_SYS_HAS__ALLOCA)

endmacro(janus_probe_functions)

##########################################################################
#                             Types Tests                                #
##########################################################################
macro(janus_test_TYPE type headers)
  set(program "")

  # Include headers.
  foreach(header ${headers})

    string(REPLACE "." "_" header_var ${header})
    string(REPLACE "/" "_" header_var ${header_var})
    string(REPLACE "-" "_" header_var ${header_var})
    string(TOUPPER ${header_var} header_var)
    set(header_var "JANUS_SYS_HAS_${header_var}")

    if(${header_var})
      set(program "${program}#include <${header}>\n")
    endif(${header_var})
  endforeach(header)

  # Begin main.
  set(program "${program}\nint main(void) { ${type} x; return 0;}\n")

  string(REPLACE "." "_" output "JANUS_SYS_HAS_${type}")
  string(REPLACE "/" "_" output ${output})
  string(REPLACE "-" "_" output ${output})
  string(REPLACE " " "_" output ${output})
  string(REPLACE "::" "_" output ${output})
  string(REGEX REPLACE "<.*>" "" output ${output})
  string(TOUPPER ${output} output)

  # Compile.
  if(NOT ${output})
    check_c_source_compiles("${program}" ${output})
  endif(NOT ${output})

  if(${output})
    set(JANUS_SYS_TYPES "${JANUS_SYS_TYPES}#define ${output}\n")
  else(${output})
    set(JANUS_SYS_TYPES "${JANUS_SYS_TYPES}//#undef ${output}\n")
  endif(${output})
endmacro(janus_test_TYPE)

macro(janus_probe_types)
  message(STATUS "")
  message(STATUS "***************************************")
  message(STATUS "***         Probing Types           ***")
  message(STATUS "***************************************")

  janus_test_type("struct timespec" "time.h;pthread.h")
  janus_test_type("struct timeval" "sys/time.h;winsock2.h")
  janus_test_type("struct termios" "termios.h")
  janus_test_type("struct stat" "sys/stat.h;sys/types.h")
  janus_test_type("struct flock" "fcntl.h")
  janus_test_type("CRITICAL_SECTION" "windows.h")
  janus_test_type("std::tr1::shared_ptr<int>" "memory;tr1/memory")
endmacro(janus_probe_types)

# Test system programs. The following variables are set to the program
# path (if any): JANUS_PROGRAM_DOXYGEN
macro(janus_probe_programs)
  message(STATUS "")
  message(STATUS "***************************************")
  message(STATUS "***        Probing Programs         ***")
  message(STATUS "***************************************")

  # Doxygen
  message(STATUS "Looking for Doxygen")
  include(FindDoxygen)

  if(DOXYGEN)
    set(JANUS_PROGRAM_DOXYGEN ${DOXYGEN})
    message(STATUS "Source documentation enabled")
  endif(DOXYGEN)

  # Info-ZIP
  message(STATUS "Looking for Info-ZIP")
  FIND_PROGRAM(ZIP_EXE NAMES zip)
  if(ZIP_EXE)
    set(JANUS_PROGRAM_ZIP ${ZIP_EXE})
    message(STATUS "Documentation packaging enabled")
  endif(ZIP_EXE)

  # Uncrustify
  message(STATUS "Looking for Uncrustify")
  FIND_PROGRAM(UNCRUSTIFY_EXE NAMES uncrustify)
  if(UNCRUSTIFY_EXE)
    set(JANUS_PROGRAM_UNCRUSTIFY ${UNCRUSTIFY_EXE})
    message(STATUS "Source code beautification enabled")
  endif(UNCRUSTIFY_EXE)
endmacro(janus_probe_programs)

macro(JANUS_PROBE_SYSTEM)
  janus_probe_cxx()
  janus_probe_cpu()
  janus_probe_os()
  janus_probe_cxx_lib()

  set(JANUS_SYSTEM_NAME
    "${JANUS_CPU_CANONICAL}-${JANUS_CPU_BITS}bit-${JANUS_OS_CANONICAL}-${JANUS_CLIB_CANONICAL}-${JANUS_CXX_CANONICAL}")

  if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/external/libraries/${JANUS_SYSTEM_NAME}/lib)
    link_directories(${PROJECT_SOURCE_DIR}/external/libraries/${JANUS_SYSTEM_NAME}/lib)
  endif(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/external/libraries/${JANUS_SYSTEM_NAME}/lib)

  set(CMAKE_REQUIRED_FLAGS "-L\"${PROJECT_SOURCE_DIR}/external/libraries/${JANUS_SYSTEM_NAME}/lib\"" "-I\"${PROJECT_SOURCE_DIR}/external/libraries/${JANUS_SYSTEM_NAME}/include\"")
  set(CMAKE_REQUIRED_INCLUDES ${PROJECT_SOURCE_DIR}/external/libraries/${JANUS_SYSTEM_NAME}/include)

  janus_probe_libs()
  janus_probe_headers()
  janus_probe_functions()
  janus_probe_types()

  message(STATUS "")
  message(STATUS "***************************************")
  message(STATUS "***    Probing Optional Libraries    **")
  message(STATUS "***************************************")

  file(GLOB cmake_includes ${PROJECT_SOURCE_DIR}/cmake/libraries/*.cmake)
  list(SORT cmake_includes)
  foreach(cmake_include ${cmake_includes})
    include(${cmake_include})
  endforeach(cmake_include ${cmake_includes})

  set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})
  set(CMAKE_MODULE_PREFIX "${PROJECT_SOURCE_DIR}/cmake/modules/Find")
  set(CMAKE_MODULE_SUFFIX ".cmake")
  file(GLOB cmake_modules ${CMAKE_MODULE_PREFIX}*${CMAKE_MODULE_SUFFIX})
  foreach(cmake_module ${cmake_modules})
    string(REGEX REPLACE ${CMAKE_MODULE_PREFIX} "" cmake_module ${cmake_module})
    string(REGEX REPLACE ${CMAKE_MODULE_SUFFIX} "" cmake_module ${cmake_module})
    find_package(${cmake_module})
  endforeach(cmake_module ${cmake_modules})

  janus_probe_programs()

  message(STATUS "")
  message(STATUS "***************************************")
  message(STATUS "***         System Sumary           ***")
  message(STATUS "***************************************")
  message(STATUS "Processor     : ${JANUS_CPU_NAME} ${JANUS_CPU_BITS} bit / ${JANUS_CPU_ENDIANNESS}")
  message(STATUS "System        : ${JANUS_OS_NAME}")
  message(STATUS "Compiler      : ${JANUS_CXX_NAME}")
  message(STATUS "C/C++ Library : ${JANUS_CLIB_NAME}")
  message(STATUS "Canonical     : ${JANUS_SYSTEM_NAME}")

  set(JANUS_PROBE_SYSTEM_DONE 1)
endmacro(JANUS_PROBE_SYSTEM)

# Force system probe for remaining macros.
if(NOT JANUS_PROBE_SYSTEM)
  JANUS_PROBE_SYSTEM()
endif(NOT JANUS_PROBE_SYSTEM)

# Utility macro to convert and escape paths on Microsoft Windows
macro(janus_set_path var path)
  if(JANUS_OS_WINDOWS AND CMAKE_HOST_WIN32)
    string(REPLACE "/" "\\\\" norm_path ${path})
    set(${var} ${norm_path})
  else(JANUS_OS_WINDOWS AND CMAKE_HOST_WIN32)
    set(${var} ${path})
  endif(JANUS_OS_WINDOWS AND CMAKE_HOST_WIN32)
endmacro(janus_set_path)

macro(janus_option name description)
  set(${name} ${${name}} CACHE BOOL description)

  if(${name})
    message(STATUS "Module ${name}: Enabled")
    set(JANUS_USING_${name} 1)
  else(${name})
    message(STATUS "Module ${name}: Disabled")
  endif(${name})
endmacro(janus_option name description)

# Create targets for generating source files.
add_custom_target(generated)

# Utility macro to retrieve the correct path for a generated source file.
macro(JANUS_GET_GENERATED_PATH output file)
  get_filename_component(outpath ${file} PATH)
  set(outpath ${JANUS_GENERATED}/${outpath})
  file(MAKE_DIRECTORY ${outpath})
  set(${output} ${outpath})
endmacro(JANUS_GET_GENERATED_PATH file)

# if(QT_LIBRARIES)
#   if(JANUS_OS_WINDOWS AND JANUS_CXX_GNU)
#     get_filename_component(JANUS_QT_PATH ${QT_QTGUI_LIBRARY} PATH)
#     set(JANUS_QT_PATH ${JANUS_QT_PATH}/../bin)
#     install(FILES ${JANUS_QT_PATH}/mingwm10.dll
#       ${JANUS_QT_PATH}/QtCore4.dll
#       ${JANUS_QT_PATH}/QtGui4.dll
#       DESTINATION bin)

#     set(QT_LIBRARIES ${QT_LIBRARIES} -mwindows)
#   endif(JANUS_OS_WINDOWS AND JANUS_CXX_GNU)
# endif(QT_LIBRARIES)

# macro(janus_qt4_wrap_ui outfiles)
#  QT4_EXTRACT_OPTIONS(ui_files ui_options ${ARGN})

#   foreach(it ${ui_files})
#     JANUS_GET_GENERATED_PATH(outpath ${it})
#     get_filename_component(outfile ${it} NAME_WE)
#     get_filename_component(infile ${it} ABSOLUTE)

#     set(outfile ${outpath}/ui_${outfile}.hpp)
#     add_custom_command(OUTPUT ${outfile}
#       COMMAND ${QT_UIC_EXECUTABLE}
#       ARGS ${ui_options} -o ${outfile} ${infile}
#       MAIN_DEPENDENCY ${infile})
#     set(${outfiles} ${${outfiles}} ${outfile})
#   endforeach(it)
# endmacro(janus_qt4_wrap_ui)

# macro(janus_qt4_wrap_cpp outfiles)
#   QT4_EXTRACT_OPTIONS(moc_files moc_options ${ARGN})

#   foreach(it ${moc_files})
#     JANUS_GET_GENERATED_PATH(outpath ${it})
#     get_filename_component(outfile ${it} NAME_WE)
#     set(outfile ${outpath}/moc_${outfile}.cpp)
#     get_filename_component(it ${it} ABSOLUTE)

#     QT4_CREATE_MOC_COMMAND(${it}  ${outfile} "" "${moc_options}")
#     set(${outfiles} ${${outfiles}} ${outfile})
#   endforeach(it)
# endmacro(janus_qt4_wrap_cpp)
