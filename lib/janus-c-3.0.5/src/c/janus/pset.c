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
// Authors: Ricardo Martins, Luigi Elia D'Amaro                           *
//*************************************************************************

// ISO C headers.
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// JANUS headers.
#include <janus/defaults.h>
#include <janus/error.h>
#include <janus/pset.h>
#include <janus/dump.h>
#include <janus/utils/memory.h>

static void
pset_compute(janus_pset_t pset)
{
  pset->chip_frq = (unsigned)JANUS_ROUND((pset->abwidth / (janus_real_t)(JANUS_CHIP_FRQ_COUNT * JANUS_ALPHABET_SIZE)));
  pset->chip_dur = pset->chip_len_mul / pset->chip_frq;

  janus_primitive(&pset->primitive, (unsigned)(pset->abwidth / (pset->chip_frq * JANUS_ALPHABET_SIZE)));
  pset->frq_block_count = pset->primitive.q - 1;
  pset->ubwidth = pset->frq_block_count * pset->chip_frq * JANUS_ALPHABET_SIZE;
}

janus_pset_t
janus_pset_new(void)
{
  janus_pset_t pset = JANUS_UTILS_MEMORY_NEW_ZERO(struct janus_pset, 1);
  strcpy(pset->name, "Custom");

  janus_pset_set_chip_len_exp(pset, 0);
  janus_pset_set_32_chip_sequence(pset, JANUS_32_CHIP_SEQUENCE);

  return pset;
}

void
janus_pset_free(janus_pset_t pset)
{
  JANUS_UTILS_MEMORY_FREE(pset);
}

int
janus_pset_load(janus_pset_t pset, const char* file, unsigned id)
{
  janus_pset_t p = janus_pset_new();
  int rv = JANUS_ERROR_NOT_FOUND;
  int matches = 0;
  FILE* fd = fopen(file, "r");

  if (fd == NULL)
    return JANUS_ERROR_FILE;

  while (!feof(fd))
  {
    matches = fscanf(fd, "%u , %u , %u , %35[^\n]\n",
                     &p->id, &p->cfreq,
                     &p->abwidth, p->name);

    if (matches != 4)
    {
      matches = fscanf(fd, "%*[^\n]\n");
      continue;
    }

    if (id == p->id)
    {
      janus_pset_set_id(pset, p->id);
      janus_pset_set_cfreq(pset, p->cfreq);
      janus_pset_set_bwidth(pset, p->abwidth);
      janus_pset_set_name(pset, p->name);
      rv = JANUS_ERROR_NONE;
      break;
    }
  }

  janus_pset_free(p);
  fclose(fd);
  return rv;
}

void
janus_pset_set_bwidth(janus_pset_t pset, unsigned value)
{
  pset->abwidth = value;
  if (janus_pset_is_valid(pset))
    pset_compute(pset);
}

unsigned
janus_pset_get_bwidth(janus_pset_t pset)
{
  return pset->abwidth;
}

void
janus_pset_set_cfreq(janus_pset_t pset, unsigned value)
{
  pset->cfreq = value;
  if (janus_pset_is_valid(pset))
    pset_compute(pset);
}

unsigned
janus_pset_get_cfreq(janus_pset_t pset)
{
  return pset->cfreq;
}

void
janus_pset_set_id(janus_pset_t pset, unsigned value)
{
  pset->id = value;
}

unsigned
janus_pset_get_id(janus_pset_t pset)
{
  return pset->id;
}

void
janus_pset_set_chip_len_exp(janus_pset_t pset, janus_uint16_t chip_len_exp)
{
  pset->chip_len_mul = JANUS_POW(JANUS_REAL_CONST(2.0), chip_len_exp);
  if (janus_pset_is_valid(pset))
    pset_compute(pset);
}

void
janus_pset_set_32_chip_sequence(janus_pset_t pset, janus_uint32_t sequence)
{
  pset->c32_sequence = sequence;
}

void
janus_pset_set_name(janus_pset_t pset, const char* name)
{
  strcpy(pset->name, name);
}

int
janus_pset_is_valid(janus_pset_t pset)
{
  if (pset->cfreq == 0 || pset->abwidth == 0)
    return 0;
  return 1;
}

void
janus_pset_dump(const janus_pset_t pset)
{
  char c32_sequence[33];
  unsigned i;

  JANUS_DUMP("Parameter Set", "Id", "%u", pset->id);
  JANUS_DUMP("Parameter Set", "Name", "%s", pset->name);
  JANUS_DUMP("Parameter Set", "Center Frequency (Hz)", "%u", pset->cfreq);
  JANUS_DUMP("Parameter Set", "Available Bandwidth (Hz)", "%u", pset->abwidth);
  JANUS_DUMP("Parameter Set", "Used Bandwidth (Hz)", "%u", pset->ubwidth);
  JANUS_DUMP("Parameter Set", "Chip Frequency (Hz)", "%u", pset->chip_frq);
  JANUS_DUMP("Parameter Set", "Chip Duration (s)", "%0.6f", pset->chip_dur);
  JANUS_DUMP("Parameter Set", "Chip Length Multiplier", "%f", pset->chip_len_mul);
  JANUS_DUMP("Parameter Set", "Block Count", "%u", pset->frq_block_count);
  JANUS_DUMP("Parameter Set", "Primitive - Q", "%u", pset->primitive.q);
  JANUS_DUMP("Parameter Set", "Primitive - Alpha", "%u", pset->primitive.alpha);
  JANUS_DUMP("Parameter Set", "Initial sequence of 32 chips (hex)", "%X", pset->c32_sequence);

  for (i = 0; i < 32; i++)
  {
    int j = ((pset->c32_sequence) & (1 << (31 - i))) ? 1 : 0;
    c32_sequence[i] = (j == 0) ? '0' : '1';
  }
  c32_sequence[32] = '\0';
  JANUS_DUMP("Parameter Set", "Initial sequence of 32 chips (binary)", "%s", c32_sequence);
}
