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

#ifndef MEX_UTILS_H_INCLUDED_
#define MEX_UTILS_H_INCLUDED_

// ISO C headers.
#include <string.h>

// Mex headers.
#include "mex.h"

// JANUS headers.
#include <janus/defaults.h>

// CLI headers.
#include <cli/options.h>

#define fprintf(output, ...) mexPrintf(__VA_ARGS__)

int
mex_utils_is_scalar(int nargin, int iargin, const mxArray* pargin[]);

int
mex_utils_is_string(int nargin, int iargin, const mxArray* pargin[]);

void
mex_utils_check_scalar(int nargin, int iargin, const mxArray* pargin[]);

void
mex_utils_check_string(int nargin, int iargin, const mxArray* pargin[]);

void
mex_utils_get_scalar(int nargin, int iargin, const mxArray* pargin[], struct cli_option* option);

void
mex_utils_get_string(int nargin, int iargin, const mxArray* pargin[], struct cli_option* option);

#endif
