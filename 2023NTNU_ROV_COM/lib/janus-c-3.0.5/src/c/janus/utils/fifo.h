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

#ifndef JANUS_UTILS_FIFO_H_INCLUDED_
#define JANUS_UTILS_FIFO_H_INCLUDED_

// JANUS headers.
#include <janus/export.h>

//! @ingroup UTILS
//! @{

//! First In, First Out data structure.
typedef struct janus_utils_fifo* janus_utils_fifo_t;

//! Create a FIFO object.
//! @param capacity FIFO capacity in bytes.
//! @return FIFO object.
JANUS_EXPORT janus_utils_fifo_t
janus_utils_fifo_new(unsigned capacity);

//! Free FIFO object.
//! @param fifo FIFO object.
JANUS_EXPORT void
janus_utils_fifo_free(janus_utils_fifo_t fifo);

//! Set FIFO size to zero.
//! @param fifo FIFO object.
JANUS_EXPORT void
janus_utils_fifo_reset(janus_utils_fifo_t fifo);

//! Retrieve current FIFO size.
//! @param fifo FIFO object.
//! @return FIFO size in bytes.
JANUS_EXPORT unsigned
janus_utils_fifo_get_size(const janus_utils_fifo_t fifo);

//! Retrieve FIFO capacity.
//! @param fifo FIFO object.
//! @return FIFO capacity in bytes.
JANUS_EXPORT unsigned
janus_utils_fifo_get_capacity(const janus_utils_fifo_t fifo);

//! Retrieve current FIFO available space.
//! @param fifo FIFO object.
//! @return FIFO available space in bytes.
JANUS_EXPORT unsigned
janus_utils_fifo_get_available(const janus_utils_fifo_t fifo);

//! Test if FIFO is empty.
//! @param fifo FIFO object.
//! @return 1 if FIFO is empty, 0 otherwise.
JANUS_EXPORT unsigned
janus_utils_fifo_is_empty(const janus_utils_fifo_t fifo);

//! Test if FIFO is full.
//! @param fifo FIFO object.
//! @return 1 if FIFO is full, 0 otherwise.
JANUS_EXPORT unsigned
janus_utils_fifo_is_full(const janus_utils_fifo_t fifo);

//! Write data to FIFO.
//! @param fifo FIFO object.
//! @param data data to insert.
//! @param data_size size of data in bytes.
//! @return amount of bytes written.
JANUS_EXPORT unsigned
janus_utils_fifo_put(janus_utils_fifo_t fifo, const void* data, unsigned data_size);

JANUS_EXPORT unsigned
janus_utils_fifo_get(janus_utils_fifo_t fifo, void* data, unsigned data_size);

JANUS_EXPORT unsigned
janus_utils_fifo_peek(const janus_utils_fifo_t fifo, void* data, unsigned data_size);

JANUS_EXPORT unsigned
janus_utils_fifo_peek_offset(const janus_utils_fifo_t fifo, void* data, unsigned data_size, unsigned offset);

JANUS_EXPORT unsigned
janus_utils_fifo_skip(janus_utils_fifo_t fifo, unsigned size);

//! @}

#endif
