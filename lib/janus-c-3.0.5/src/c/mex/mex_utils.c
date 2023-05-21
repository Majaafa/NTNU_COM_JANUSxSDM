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

// Local headers.
#include <mex/mex_utils.h>

int
mex_utils_is_scalar(int nargin, int iargin, const mxArray* pargin[])
{
  if (iargin < nargin && mxIsNumeric(pargin[iargin]))
  {
    return 1;
  }

  return 0;
}

int
mex_utils_is_string(int nargin, int iargin, const mxArray* pargin[])
{
  if (iargin < nargin && mxIsChar(pargin[iargin]))
  {
    return 1;
  }

  return 0;
}

void
mex_utils_check_scalar(int nargin, int iargin, const mxArray* pargin[])
{
  if (iargin < nargin && !mex_utils_is_scalar(nargin, iargin, pargin))
  {
    mexErrMsgIdAndTxt("MATLAB:janus_rx_mex:rhs", "Input argument %i should be numeric.", iargin + 1);
  }
}

void
mex_utils_check_string(int nargin, int iargin, const mxArray* pargin[])
{
  if (iargin < nargin && !mex_utils_is_string(nargin, iargin, pargin))
  {
    mexErrMsgIdAndTxt("MATLAB:janus_rx_mex:rhs", "Input argument %i should be a string.", iargin + 1);
  }
}

void
mex_utils_get_scalar(int nargin, int iargin, const mxArray* pargin[], struct cli_option* option)
{
  if (iargin < nargin)
  {
    char value[ARG_LEN];
    sprintf(value, "%u", (unsigned)mxGetScalar(pargin[iargin]));
    strncpy((char*)option->arg, value, ARG_LEN - 1);
    option->arg[ARG_LEN - 1] = '\0';
    option->is_present = 1;
  }
}

void
mex_utils_get_string(int nargin, int iargin, const mxArray* pargin[], struct cli_option* option)
{
  if (iargin < nargin)
  {
    char value[ARG_LEN];
    int rv = mxGetString(pargin[iargin], value, ARG_LEN);
    if (rv == 1)
    {
      printf("ERROR: failed to read input argument %d\n", iargin + 1);
      return;
    }
    strncpy((char*)option->arg, value, ARG_LEN - 1);
    option->arg[ARG_LEN - 1] = '\0';
    option->is_present = 1;
  }
}
