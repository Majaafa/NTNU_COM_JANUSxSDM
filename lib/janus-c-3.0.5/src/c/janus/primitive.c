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

#include <math.h>
#include <stdio.h>

#include <janus/primitive.h>

static const struct janus_primitive c_primitives_table[] =
{
  {0, 0}, {0, 0}, {3, 2}, {3, 2}, {5, 2}, {5, 2},
  {7, 3}, {7, 3}, {7, 3}, {7, 3}, {11, 2}, {11, 2},
  {13, 2}, {13, 2}, {13, 2}, {13, 2}, {17, 3}, {17, 3},
  {19, 2}, {19, 2}, {19, 2}, {19, 2}, {23, 5}, {23, 5},
  {23, 5}, {23, 5}, {23, 5}, {23, 5}, {29, 2}, {29, 2},
  {31, 3}, {31, 3}, {31, 3}
};

void
janus_primitive(janus_primitive_t primitive, unsigned nblock)
{
  primitive->q = c_primitives_table[nblock].q;
  primitive->alpha = c_primitives_table[nblock].alpha;
}
