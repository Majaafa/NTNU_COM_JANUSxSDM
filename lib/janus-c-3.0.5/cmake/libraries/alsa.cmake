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

janus_test_lib(asound snd_pcm_open)
janus_test_header_deps(alsa/asoundlib.h "")
if(JANUS_SYS_HAS_LIB_ASOUND AND JANUS_SYS_HAS_ALSA_ASOUNDLIB_H)
  set(JANUS_WITH_ALSA 1 CACHE INTERNAL "ALSA")
endif(JANUS_SYS_HAS_LIB_ASOUND AND JANUS_SYS_HAS_ALSA_ASOUNDLIB_H)
