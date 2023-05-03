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

#ifndef JANUS_H_INCLUDED_
#define JANUS_H_INCLUDED_

#include <janus/config.h>
#include <janus/constants.h>
#include <janus/error.h>

//! @defgroup TYPES Basic Types
#include <janus/types.h>
#include <janus/complex.h>

//! @defgroup PSET Parameter Set Handling
#include <janus/pset.h>
#include <janus/pset_parser.h>

//! @defgroup PARAMETERS Parameters
#include <janus/parameters.h>

//! @defgroup FH Frequency Hopping
#include <janus/hop_index.h>
#include <janus/primitive.h>

//! @defgroup CONV Convolutional Coding and Decoding
#include <janus/trellis.h>
#include <janus/viterbi.h>
#include <janus/convolve.h>

//! @defgroup MODEM Modulation and Demodulation
#include <janus/modulator.h>
#include <janus/demodulator.h>

//! @defgroup HETERODYNE Heterodyning
#include <janus/ddc.h>
#include <janus/duc.h>

//! @defgroup IDLEAVER Interleaving and Deinterleaving
#include <janus/interleave.h>
#include <janus/deinterleave.h>

//! @defgroup WUT Wake-up Tones Generation
#include <janus/wake_up_tones.h>

//! @defgroup STREAMS Streams
#include <janus/stream/format.h>
#include <janus/stream/istream.h>
#include <janus/stream/ostream.h>

//! @defgroup PACKET Packet Transmission and Reception
#include <janus/packet.h>
#include <janus/crc.h>
#include <janus/tx.h>
#include <janus/rx.h>
#include <janus/simple_tx.h>
#include <janus/simple_rx.h>
#include <janus/carrier_sensing.h>

#endif

/*!
  \mainpage Developer's Guide

  \section Prerequisites

  The JANUS C reference implementation requires the following tools:

  \li CMake version 2.6 or greater. (http://www.cmake.org)
  \li FFTW version 3. (http://www.fftw.org)
  \li C and C++ Compiler.

  \subsection ins_ubuntu Building on Ubuntu

  \subsubsection ins_ubuntu_deps Install required dependencies:

\verbatim
$ apt-get install cmake libfftw3-dev libasound-dev gcc g++
\endverbatim

  \subsubsection ins_ubuntu_prep Preparing build folders

\verbatim
$ mkdir -p "$HOME/janus/build"
$ cd "$HOME/janus" && tar xvf janus-<version>.tar.bz2
\endverbatim

  \subsubsection ins_ubuntu_conf Configuring and compiling

\verbatim
$ cd "$HOME/janus/build"
$ cmake ../janus && make
\endverbatim

  To rebuild the source code after changing some file just run "make".

\section janus_tx Creating a JANUS waveform

\verbatim
$ ./janus-tx \
  --pset-file ../janus-<version>/etc/parameter_sets.csv \
  --pset-id 1 \
  --ostream-driver wav \
  --ostream-driver-args janus.wav
\endverbatim

*/
