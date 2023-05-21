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
# Get SVN revision of source tree.                                       #
##########################################################################

execute_process(COMMAND svnversion -c
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
  OUTPUT_VARIABLE PROJECT_SVN_REVISION
  ERROR_QUIET
  OUTPUT_STRIP_TRAILING_WHITESPACE)

string(REGEX REPLACE "^[^:]+:" "" PROJECT_SVN_REVISION "${PROJECT_SVN_REVISION}")

string(COMPARE EQUAL "${PROJECT_SVN_REVISION}" "" empty_string)
if(empty_string)
  set(PROJECT_SVN_REVISION "unknown")
endif(empty_string)
