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

janus_test_lib(pulse-simple pa_simple_new)
janus_test_lib(pulse pa_strerror)
janus_test_header_deps(pulse/simple.h "")
if(JANUS_SYS_HAS_LIB_PULSE_SIMPLE AND JANUS_SYS_HAS_PULSE_SIMPLE_H)
  set(JANUS_WITH_PULSE 1 CACHE INTERNAL "PULSE")
endif(JANUS_SYS_HAS_LIB_PULSE_SIMPLE AND JANUS_SYS_HAS_PULSE_SIMPLE_H)
