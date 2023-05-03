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

// ISO C headers.
#include <stdio.h>
#include <string.h>

// JANUS headers.
#include <janus_tukey_window.h>

int
main(int argc, char** argv)
{
  // DAT filename.
  char fname[256];
  // Read value (pre-computed).
  char rvalue[16];
  // Computed value.
  char cvalue[16];
  // Scratch buffer to hold the result of computations.
  complex_t tmp[250] = {0};
  
  for (unsigned n = 200; n < 251; ++n)
  {
    for (double r = 0.05; r < 1.05; r += 0.05)
    {
      janus_utils_tukey_window(n, r, tmp);
      sprintf(fname, "dat/tukey-%u-%0.3f.dat", n, r);
      FILE* fd = fopen(fname, "r");
      for (unsigned i = 0; i < n; ++i)
      {
        fscanf(fd, "%[^\n]\n", rvalue);
        sprintf(cvalue, "%0.8f", creal(tmp[i]));
        
        if (strcmp(rvalue, cvalue) != 0)
        {
          fprintf(stderr, "ERROR: mismatch in '%s' at line %d: %s != %s\n", fname, i, cvalue, rvalue);
          return 1;
        }
      }
      fclose(fd);
    }
  }

  return 0;
}
