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

#ifndef JANUS_UTILS_GO_CFAR_H_INCLUDED_
#define JANUS_UTILS_GO_CFAR_H_INCLUDED_

// JANUS headers.
#include <janus/types.h>
#include <janus/utils/fifo.h>

typedef struct janus_go_cfar* janus_go_cfar_t;

janus_go_cfar_t
janus_go_cfar_new(unsigned n, unsigned g, janus_real_t w, unsigned s, unsigned c, unsigned o);

void
janus_go_cfar_free(janus_go_cfar_t go_cfar);

void
janus_go_cfar_reset(janus_go_cfar_t go_cfar);

unsigned
janus_go_cfar_execute(janus_go_cfar_t go_cfar, janus_utils_fifo_t fifo);

int
janus_go_cfar_has_detected(janus_go_cfar_t go_cfar);

int
janus_go_cfar_detection_competed(janus_go_cfar_t go_cfar);

unsigned
janus_go_cfar_get_window_size(janus_go_cfar_t go_cfar);

janus_real_t
janus_go_cfar_get_detection_offset(janus_go_cfar_t go_cfar);

janus_uint64_t
janus_go_cfar_get_counter(janus_go_cfar_t go_cfar);

janus_uint64_t 
janus_go_cfar_get_first_detection(janus_go_cfar_t go_cfar);

janus_real_t 
janus_go_cfar_get_detection_peak_energy(janus_go_cfar_t go_cfar);

janus_real_t 
janus_go_cfar_get_detection_background_energy(janus_go_cfar_t go_cfar);

void
janus_go_cfar_set_threshold(janus_go_cfar_t go_cfar, janus_real_t detection_threshold);

#endif
