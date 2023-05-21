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
// Author: Luigi Elia D'Amaro, Roberto Petroccia                          *
//*************************************************************************

// ISO C headers.
#include <stdlib.h>
#include <stdio.h>

// JANUS headers.
#include <janus/codec/plugin.h>
#include <janus/config.h>

#if (defined(_WIN32) || defined(__WIN32__))
#include <WinBase.h>
#endif

static void*
janus_plugin_get_function(janus_plugin_t plugin, const char* function_name)
{
#if (defined(_WIN32) || defined(__WIN32__))
  return GetProcAddress(plugin, function_name);
#else
  return dlsym(plugin, function_name);
#endif
}

janus_plugin_t
janus_plugin_open(const char* plugin_name)
{
#if (defined(_WIN32) || defined(__WIN32__))
  static char plugins_path_set = 0;
  if (plugins_path_set == 0)
  {
    SetDllDirectoryA(JANUS_PLUGINS_PATH);
    plugins_path_set = 1;
  }
  return LoadLibrary(plugin_name);
#else
  return dlopen(plugin_name, RTLD_NOW | RTLD_NODELETE);
#endif
}

void
janus_plugin_close(janus_plugin_t plugin)
{
#if (defined(_WIN32) || defined(__WIN32__))
  FreeLibrary(plugin);
#else
  dlclose(plugin);
#endif
}

char* 
janus_plugin_error(janus_plugin_t plugin)
{
#if (defined(_WIN32) || defined(__WIN32__))
  char* error = (char*)malloc(1024 * sizeof(char));
  LPTSTR msg;
  if (FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_FROM_HMODULE,
                   (LPCVOID)plugin, GetLastError(), 0, (LPTSTR)&msg, 0, NULL) == 0)
  {
    strcpy(error, "unknown error");
    return error;
  }
  strcpy(error, msg);
  LocalFree(msg);
  return error;
#else
  return dlerror();
#endif
}

janus_app_data_decode_function_t
janus_plugin_get_app_data_decode_function(janus_plugin_t plugin)
{
  void* function_address = NULL;
  
  if (plugin != NULL)
  {
    function_address = janus_plugin_get_function(plugin, "app_data_decode");

#if !(defined(_WIN32) || defined(__WIN32__))
    dlclose(plugin);
#endif
  }

  return *((janus_app_data_decode_function_t*)(&function_address));
}

janus_app_data_encode_function_t
janus_plugin_get_app_data_encode_function(janus_plugin_t plugin)
{
  void* function_address = NULL;
  
  if (plugin != NULL)
  {
    function_address = janus_plugin_get_function(plugin, "app_data_encode");

#if !(defined(_WIN32) || defined(__WIN32__))
    dlclose(plugin);
#endif
  }

  if (function_address == NULL)
    return NULL;
  else
    return *((janus_app_data_encode_function_t*)(&function_address));
}

janus_cargo_decode_function_t
janus_plugin_get_cargo_decode_function(janus_plugin_t plugin)
{
  void* function_address = NULL;
  
  if (plugin != NULL)
  {
    function_address = janus_plugin_get_function(plugin, "cargo_decode");

#if !(defined(_WIN32) || defined(__WIN32__))
    dlclose(plugin);
#endif
  }

  return *((janus_cargo_decode_function_t*)(&function_address));
}

janus_cargo_encode_function_t
janus_plugin_get_cargo_encode_function(janus_plugin_t plugin)
{
  void* function_address = NULL;
  if (plugin != NULL)
  {
    function_address = janus_plugin_get_function(plugin, "cargo_encode");

#if !(defined(_WIN32) || defined(__WIN32__))
    dlclose(plugin);
#endif
  }

  if (function_address == NULL)
    return NULL;
  else
    return *((janus_cargo_encode_function_t*)(&function_address));
}
