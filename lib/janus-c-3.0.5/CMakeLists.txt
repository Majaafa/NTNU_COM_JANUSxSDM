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
# Authors: Ricardo Martins, Luigi Elia D'Amaro                           #
##########################################################################

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

include(cmake/Toolchain.cmake)

project("JANUS")
set(PROJECT_NAME "JANUS")
set(PROJECT_SHORT_NAME "JANUS")
set(PROJECT_VENDOR "STO Centre for Maritime Research and Experimentation (CMRE)")
set(PROJECT_COPYRIGHT "Copyright (C) 2008-2018 - ${PROJECT_VENDOR}")
set(PROJECT_VERSION_MAJ "3")
set(PROJECT_VERSION_MIN "0")
set(PROJECT_VERSION_PAT "5")

set(JANUS_GENERATED "${PROJECT_BINARY_DIR}/cmake_generated")

# Version string.
set(PROJECT_VERSION
  "${PROJECT_VERSION_MAJ}.${PROJECT_VERSION_MIN}.${PROJECT_VERSION_PAT}")

include(cmake/Macros.cmake)
include(cmake/modules/Subversion.cmake)

configure_file(
  "${PROJECT_SOURCE_DIR}/src/c/janus/config.h.in"
  "${JANUS_GENERATED}/janus/config.h")

include_directories(
  "${PROJECT_SOURCE_DIR}/src/c"
  "${JANUS_GENERATED}")

# Source files.
file(GLOB_RECURSE JANUS_SRCS ${PROJECT_SOURCE_DIR}/src/c/janus/*.c)
file(GLOB_RECURSE JANUS_HDRS ${PROJECT_SOURCE_DIR}/src/c/janus/*.h)
if(NOT ${JANUS_CXX_FLAGS_STRICT} STREQUAL "")
  set_source_files_properties(${JANUS_SRCS} PROPERTIES
    COMPILE_FLAGS ${JANUS_CXX_FLAGS_STRICT})
endif(NOT ${JANUS_CXX_FLAGS_STRICT} STREQUAL "")

##########################################################################
#                               Libraries                                #
##########################################################################
set(JANUS_LIB janus)
add_library(${JANUS_LIB} STATIC ${JANUS_SRCS} ${JANUS_HDRS})

if(NOT FFTW3_FOUND)
  message(FATAL_ERROR "FTTW3 required!")
endif(NOT FFTW3_FOUND)

include_directories(${FFTW3_INCLUDE_DIRS})
set(JANUS_SYS_LIBS ${JANUS_SYS_LIBS} ${FFTW3_LIBRARIES})

if(WIN32)
  set_target_properties(${JANUS_LIB} PROPERTIES
    COMPILE_FLAGS "-D_WIN32_WINNT=0x0502")
endif(WIN32)

##########################################################################
#                              Executables                               #
##########################################################################
set(JANUS_CLI_SRCS
  src/c/cli/string.h
  src/c/cli/options.h
  src/c/cli/string.c
  src/c/cli/options.c)
if(NOT ${JANUS_CXX_FLAGS_STRICT} STREQUAL "")
  set_source_files_properties(${JANUS_CLI_SRCS} PROPERTIES
    COMPILE_FLAGS ${JANUS_CXX_FLAGS_STRICT})
endif(NOT ${JANUS_CXX_FLAGS_STRICT} STREQUAL "")

set(EXE_TYPES tx rx)
foreach(EXE_TYPE ${EXE_TYPES})
  set(EXE_FILE janus-${EXE_TYPE})
  set(EXE_FLAG "-DCLI_${EXE_TYPE}")
  string(TOUPPER ${EXE_FLAG} EXE_FLAG)

  add_executable(${EXE_FILE} src/c/cli/main.c ${JANUS_CLI_SRCS})
  target_link_libraries(${EXE_FILE} ${JANUS_LIB} ${JANUS_SYS_LIBS})
  set_target_properties(${EXE_FILE} PROPERTIES
    COMPILE_FLAGS ${EXE_FLAG}
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/share/janus/plugins")
endforeach(EXE_TYPE ${EXE_TYPES})

##########################################################################
#                                Plugins                                 #
##########################################################################
set(JANUS_PLUGINS_PATH ${CMAKE_INSTALL_PREFIX}/share/janus/plugins)
add_definitions( -DJANUS_PLUGINS_PATH="${JANUS_PLUGINS_PATH}" )
if(${JANUS_PLUGINS_DEBUG})
  add_definitions( -DJANUS_PLUGINS_DEBUG="${JANUS_PLUGINS_DEBUG}" )
else(${JANUS_PLUGINS_DEBUG})
  remove_definitions( -DJANUS_PLUGINS_DEBUG )
endif(${JANUS_PLUGINS_DEBUG})
file(GLOB JANUS_PLUGIN_SRCS ${PROJECT_SOURCE_DIR}/src/c/plugins/plugin_*_*.c)
foreach(JANUS_PLUGIN_SRC ${JANUS_PLUGIN_SRCS})
  get_filename_component(JANUS_PLUGIN_PATH ${JANUS_PLUGIN_SRC} PATH)
  get_filename_component(JANUS_PLUGIN_NAME ${JANUS_PLUGIN_SRC} NAME_WE)
  message ("path ${JANUS_PLUGIN_PATH} name ${JANUS_PLUGIN_NAME}")

  if(NOT ${JANUS_CXX_FLAGS_STRICT} STREQUAL "")
    set_source_files_properties(${JANUS_PLUGINS_SRCS} PROPERTIES
      COMPILE_FLAGS ${JANUS_CXX_FLAGS_STRICT})
  endif(NOT ${JANUS_CXX_FLAGS_STRICT} STREQUAL "")

  add_library(${JANUS_PLUGIN_NAME} MODULE ${JANUS_PLUGIN_SRC})
  target_link_libraries(${JANUS_PLUGIN_NAME} ${JANUS_LIB})

  if(UNIX)
    set_target_properties(${JANUS_PLUGIN_NAME} PROPERTIES
      LINK_FLAGS -nostdlib)
  endif(UNIX)

  install(TARGETS ${JANUS_PLUGIN_NAME}
    LIBRARY DESTINATION ${JANUS_PLUGINS_PATH})
endforeach(JANUS_PLUGIN_SRC)

##########################################################################
#                               Mex files                                #
##########################################################################
if(MATLAB_FOUND AND MATLAB_MEXFILE_EXT)
  set(JANUS_MEX_SRCS
    src/c/mex/mex_utils.h
    src/c/mex/mex_utils.c
    ${JANUS_CLI_SRCS})
  if(NOT ${JANUS_CXX_FLAGS_STRICT} STREQUAL "")
    set_source_files_properties(${JANUS_MEX_SRCS} PROPERTIES
      COMPILE_FLAGS ${JANUS_CXX_FLAGS_STRICT})
  endif(NOT ${JANUS_CXX_FLAGS_STRICT} STREQUAL "")

  set(MEX_TYPES tx rx)
  foreach(MEX_TYPE ${MEX_TYPES})
    set(MEX_FILE janus_${MEX_TYPE}_mex)
    set(MEX_FLAG "-DCLI_${MEX_TYPE}")
    string(TOUPPER ${MEX_FLAG} MEX_FLAG)
    include_directories(${MATLAB_INCLUDE_DIR})

    add_library(${MEX_FILE} SHARED src/c/mex/${MEX_FILE}.c ${JANUS_MEX_SRCS})
    target_link_libraries(${MEX_FILE} ${JANUS_LIB} ${JANUS_SYS_LIBS} ${MATLAB_LIBRARIES})

    if(WIN32)
      set_target_properties(${MEX_FILE} PROPERTIES
        COMPILE_FLAGS ${MEX_FLAG}
        LINK_FLAGS "-export:mexFunction"
        PREFIX ""
        SUFFIX ".${MATLAB_MEXFILE_EXT}")
    else(WIN32)
      set_target_properties(${MEX_FILE} PROPERTIES
        COMPILE_FLAGS ${MEX_FLAG}
        PREFIX ""
        SUFFIX ".${MATLAB_MEXFILE_EXT}")
    endif(WIN32)

    install(TARGETS ${MEX_FILE}
      DESTINATION share/janus/matlab/mex)
  endforeach(MEX_TYPE ${MEX_TYPES})

endif(MATLAB_FOUND AND MATLAB_MEXFILE_EXT)

##########################################################################
#                                 Tests                                  #
##########################################################################
add_executable(test_convolve tests/c/convolve.c)
target_link_libraries(test_convolve ${JANUS_LIB} ${JANUS_SYS_LIBS})

add_executable(test_interleave tests/c/interleaver.c)
target_link_libraries(test_interleave ${JANUS_LIB} ${JANUS_SYS_LIBS})

add_executable(test_hop_index tests/c/hop_index.c)
target_link_libraries(test_hop_index ${JANUS_LIB} ${JANUS_SYS_LIBS})

add_executable(test_wake_up_tone tests/c/wakeup_tones.c)
target_link_libraries(test_wake_up_tone ${JANUS_LIB} ${JANUS_SYS_LIBS})

add_executable(test_packet tests/c/packet.c)
target_link_libraries(test_packet ${JANUS_LIB} ${JANUS_SYS_LIBS})

##########################################################################
#                                Programs                                #
##########################################################################

add_executable(janus-dump-tables programs/dump_tables.c)
target_link_libraries(janus-dump-tables ${JANUS_LIB} ${JANUS_SYS_LIBS})

##########################################################################
#                             Documentation                              #
##########################################################################
if(JANUS_PROGRAM_DOXYGEN AND JANUS_PROGRAM_ZIP)

  foreach(source ${JANUS_HDRS})
    set(JANUS_HDRS_STR "${JANUS_HDRS_STR} ${source}")
  endforeach(source ${JANUS_HDRS})

  set(JANUS_DOCS_LABEL
    janus-${PROJECT_VERSION}-doc)

  set(JANUS_DOXYGEN_DOCS_DIR
    "${JANUS_GENERATED}/${JANUS_DOCS_LABEL}")

  configure_file(${PROJECT_SOURCE_DIR}/doc/Doxyfile.in
    "${JANUS_GENERATED}/Doxyfile")

  add_custom_target(doc
    ${JANUS_PROGRAM_DOXYGEN}
    "${JANUS_GENERATED}/Doxyfile")

  add_custom_target(doc_package
    zip -r ${CMAKE_BINARY_DIR}/${JANUS_DOCS_LABEL}.zip ${JANUS_DOCS_LABEL}
    WORKING_DIRECTORY ${JANUS_GENERATED})

  add_dependencies(doc_package doc)
endif(JANUS_PROGRAM_DOXYGEN AND JANUS_PROGRAM_ZIP)

##########################################################################
#                         Packaging/Installation                         #
##########################################################################
install(TARGETS ${JANUS_LIB} janus-tx janus-rx
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Installation of headers.
foreach(header ${JANUS_HDRS})
  string(REGEX REPLACE "${PROJECT_SOURCE_DIR}/" "" header ${header})
  string(REGEX REPLACE "src/c" "include" destination ${header})
  get_filename_component(destination ${destination} PATH)
  install(FILES ${header} DESTINATION ${destination})
endforeach(header ${JANUS_HDRS})

install(FILES "${JANUS_GENERATED}/janus/config.h"
  DESTINATION include/janus)

install(FILES "${PROJECT_SOURCE_DIR}/COPYING"
  DESTINATION .)

install(DIRECTORY etc DESTINATION share/janus
  PATTERN .svn EXCLUDE)

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${PROJECT_NAME})
set(CPACK_PACKAGE_NAME ${PROJECT_SHORT_NAME})
set(CPACK_PACKAGE_VENDOR ${PROJECT_VENDOR})
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_CONTACT ${PROJECT_CONTACT})
set(CPACK_STRIP_FILES "bin/janus-tx" "bin/janus-rx")
set(CPACK_PACKAGE_EXECUTABLES "janus-tx;JANUS TX" "janus-rx;JANUS RX")
set(CPACK_PACKAGE_FILE_NAME
  "janus-${CPACK_PACKAGE_VERSION}-${JANUS_SYSTEM_NAME}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY
  "${PROJECT_SHORT_NAME} v${PROJECT_VERSION}")
set(CPACK_RESOURCE_FILE_LICENSE
  "${PROJECT_SOURCE_DIR}/COPYING")

if(JANUS_OS_WINDOWS)
  set(CPACK_GENERATOR "ZIP")
else(JANUS_OS_WINDOWS)
  set(CPACK_GENERATOR "TBZ2")
endif(JANUS_OS_WINDOWS)

include(InstallRequiredSystemLibraries)
include(CPack)
