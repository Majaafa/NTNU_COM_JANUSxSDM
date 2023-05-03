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

janus_test_lib(FLAC FLAC__stream_encoder_process)
janus_test_header_deps(FLAC/all.h "")
if(JANUS_SYS_HAS_LIB_FLAC AND JANUS_SYS_HAS_FLAC_ALL_H)
  set(JANUS_WITH_FLAC 1 CACHE INTERNAL "FLAC: Free Lossless Audio Codec")
endif(JANUS_SYS_HAS_LIB_FLAC AND JANUS_SYS_HAS_FLAC_ALL_H)
