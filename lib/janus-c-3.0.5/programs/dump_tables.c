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
#include <errno.h>
#include <stdlib.h>
#include <time.h>

// JANUS headers.
#include <janus/janus.h>

// File name prefix.
#define FILE_PREFIX "janus-table-"

static char g_timestamp[32] = {0};

void
die(const char* message)
{
  fprintf(stderr, "ERROR: %s: ", message);
  perror("");
  exit(1);
}

void
write_header(FILE* fd)
{
  fprintf(fd, "# Source Code Revision: %s\n", JANUS_SVN_REV_STR);
  fprintf(fd, "# Created On: %s\n", g_timestamp);
}

void
fh_dump_table(unsigned alpha, unsigned blocksize, unsigned size)
{
  unsigned i = 0;
  char file[32] = {0};
  FILE* fd = NULL;

  sprintf(file, FILE_PREFIX "fh-%u-%u-%u.txt", alpha, blocksize, size);
  fd = fopen(file, "w");
  if (fd == NULL)
    die("failed to create file for FH table");

  fprintf(fd, "# FH table, Table size: %u, Alpha: %u, Block size: %u\n", size, alpha, blocksize);
  write_header(fd);
  for (i = 0; i < size; ++i)
    fprintf(fd, "%u\n", janus_hop_index(i, 2, 13));

  fclose(fd);
}

void
crc8_dump_table(void)
{
  unsigned i = 0;
  FILE* fd = fopen(FILE_PREFIX "crc8.txt", "w");
  if (fd == NULL)
    die("failed to create file for CRC8 table");

  fprintf(fd, "# CRC8, polynomial x^8+x^2+x+1\n");
  write_header(fd);

  for (i = 0; i < 256; ++i)
    fprintf(fd, "0x%02X\n", janus_crc_byte((janus_uint8_t)i, 0));

  fclose(fd);
}

int
main(void)
{
  // Get timestamp.
  time_t now = time(NULL);
  struct tm* now_tm = gmtime(&now);
  strftime(g_timestamp, sizeof(g_timestamp) - 1, "%F %R", now_tm);

  // Dump frequency hopping patterns.
  fh_dump_table(2, 13, 8192);

  // Dump CRC8 table.
  crc8_dump_table();

  return 0;
}
