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

#ifndef JANUS_EXPORT_H_INCLUDED_
#define JANUS_EXPORT_H_INCLUDED_

#ifdef __cplusplus
#  define JANUS_EXPORT extern "C"
#  if (defined(_WIN32) || defined(__WIN32__))
#    define JANUS_PLUGIN_EXPORT extern "C" __declspec(dllexport)
#  else
#    define JANUS_PLUGIN_EXPORT extern "C"
#  endif
#else
#  define JANUS_EXPORT 
#  if (defined(_WIN32) || defined(__WIN32__))
#    define JANUS_PLUGIN_EXPORT __declspec(dllexport)
#  else
#    define JANUS_PLUGIN_EXPORT 
#  endif
#endif

#ifdef _MSC_VER
#ifndef inline
#define inline __inline
#endif
#endif

#endif
