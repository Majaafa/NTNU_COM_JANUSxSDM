# - Find Matlab
# Find the Matlab headers, libraries and mex files extension.
#
# MATLAB (matrix laboratory) is a numerical computing environment and fourth-generation programming language. Please see
# http://www.mathworks.com/products/matlab/index.html
#
# This module defines the following variables:
#
#   MATLAB_INCLUDE_DIRS  - Path to find mex.h, etc.
#   MATLAB_LIBRARIES     - List of libraries.
#   MATLAB_MEXFILE_EXT   - File extention of mex files.
#   MATLAB_FOUND         - True if Matlab found.
#
# For each component the following variables are set. You can use these 
# variables if you would like to pick and choose components for your targets
# instead of just using MATLAB_LIBRARIES.
#
#   MATLAB_MAT_LIBRARY
#   MATLAB_MEX_LIBRARY
#   MATLAB_MX_LIBRARY
#

file(GLOB MATLAB_DIRS ${MATLAB_DIR} ${CMAKE_FIND_ROOT_PATH}/usr/local/matlab* ${CMAKE_FIND_ROOT_PATH}/usr/matlab* ${CMAKE_FIND_ROOT_PATH}/opt/matlab*)

# Look for the header files.
find_path(MATLAB_INCLUDE_DIR mex.h
  PATHS
    $ENV{MATLAB_DIR}
    ${MATLAB_DIRS}
  PATH_SUFFIXES
    extern/include
)

# Define a macro for searching a library.
macro(FIND_COMPONENT_LIBRARY MYLIBRARY MYLIBRARYNAME)
  find_library(${MYLIBRARY}
    NAMES ${MYLIBRARYNAME} lib${MYLIBRARYNAME}
    PATHS
      $ENV{MATLAB_DIR}
      ${MATLAB_DIRS}
    PATH_SUFFIXES
      bin
      bin/glnx86
      bin/glnxa64
      bin/win32
      bin/win64
      extern/lib/win32/microsoft
      extern/lib/win64/microsoft
      bin/mac
      bin/maci
      bin/hpux
      bin/sol2
  )
endmacro()

# Look for the libraries.
FIND_COMPONENT_LIBRARY(MATLAB_MAT_LIBRARY mat)
FIND_COMPONENT_LIBRARY(MATLAB_MEX_LIBRARY mex)
FIND_COMPONENT_LIBRARY(MATLAB_MX_LIBRARY mx)

# Look for the mexext executable file.
find_program(MATLAB_MEXEXT_EXECUTABLE
  NAMES mexext mexext.bat
  PATHS
      $ENV{MATLAB_DIR}
      ${MATLAB_DIRS}
  PATH_SUFFIXES
    bin
)

# Read mex file extension from mexext.
if(MATLAB_MEXEXT_EXECUTABLE)
  execute_process(COMMAND ${MATLAB_MEXEXT_EXECUTABLE} OUTPUT_VARIABLE MATLAB_MEXFILE_EXT)
  string(REGEX REPLACE "\n" "" MATLAB_MEXFILE_EXT ${MATLAB_MEXFILE_EXT})
endif()

# Handle the QUIETLY and REQUIRED arguments and set MATLAB_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(MATLAB DEFAULT_MSG MATLAB_INCLUDE_DIR MATLAB_MAT_LIBRARY MATLAB_MEX_LIBRARY MATLAB_MX_LIBRARY)

# Copy the results to the output variables.
if(MATLAB_FOUND)
  set(MATLAB_INCLUDE_DIRS ${MATLAB_INCLUDE_DIR})
  set(MATLAB_LIBRARIES ${MATLAB_MAT_LIBRARY} ${MATLAB_MEX_LIBRARY} ${MATLAB_MX_LIBRARY})
else()
  set(MATLAB_INCLUDE_DIRS)
  set(MATLAB_LIBRARIES)
endif()

mark_as_advanced(MATLAB_INCLUDE_DIRS MATLAB_LIBRARIES MATLAB_MAT_LIBRARY MATLAB_MEX_LIBRARY MATLAB_MX_LIBRARY MATLAB_MEXFILE_EXT)
