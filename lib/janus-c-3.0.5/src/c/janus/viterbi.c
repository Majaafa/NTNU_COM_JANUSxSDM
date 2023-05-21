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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// JANUS headers.
#include <janus/viterbi.h>
#include <janus/utils/memory.h>

#define C_MAX_METRIC 999999U

struct janus_viterbi
{
  // Trellis structure.
  janus_trellis_t trellis;
  // Soft decision bits.
  unsigned sdc_bits;
  // Branch metric.
  unsigned* met_brc;
  // Size of branch metric array.
  unsigned met_brc_size;
  // Temporary metric.
  unsigned* met_tmp;
  // Size of temporary metric array.
  unsigned met_tmp_size;
  // State metric.
  unsigned* met_sta;
  // Size of state metric array.
  unsigned met_sta_size;
  // Traceback depth.
  unsigned tbk_depth;
  // Traceback input.
  unsigned* tbk_inp;
  // Size of traceback input array.
  unsigned tbk_inp_size;
  // Traceback state.
  unsigned* tbk_sta;
  // Size of traceback state array.
  unsigned tbk_sta_size;
  // Number of bits per output symbol.
  unsigned k;
  // Number of bits per input symbol.
  unsigned n;
};

static void
reset_traceback(janus_viterbi_t vit, unsigned tbk_depth)
{
  if (vit->tbk_depth == tbk_depth)
  {
    memset(vit->tbk_inp, 0, vit->tbk_inp_size * sizeof(unsigned));
    memset(vit->tbk_sta, 0, vit->tbk_sta_size * sizeof(unsigned));
    return;
  }
  
  if (vit->tbk_inp)
    free(vit->tbk_inp);
  
  if (vit->tbk_sta)
    free(vit->tbk_sta);
  
  vit->tbk_depth = tbk_depth;
  vit->tbk_inp_size = vit->trellis->state_count * (tbk_depth + 1);
  vit->tbk_inp = JANUS_UTILS_MEMORY_NEW_ZERO(unsigned, vit->tbk_inp_size);
  vit->tbk_sta_size = vit->trellis->state_count * (tbk_depth + 1);
  vit->tbk_sta = JANUS_UTILS_MEMORY_NEW_ZERO(unsigned, vit->tbk_sta_size);
}

static void
reset_metric(janus_viterbi_t vit)
{
  memset(vit->met_sta, 0, vit->met_sta_size * sizeof(unsigned));
}

static void
branch_metric(janus_viterbi_t vit, const janus_uint8_t* inp, unsigned idx)
{
  int inp_off = idx * vit->n;
  int tmp = 0;

  int i;
  unsigned j = 0;
  for (i = 0; i < (1 << vit->n); ++i) 
  {
    tmp = i;
    vit->met_brc[i] = 0;
    
    for (j = 0; j < vit->n; ++j) 
    {
      if (tmp & 1) 
        vit->met_brc[i] += ((1 << vit->sdc_bits) - 1) - inp[inp_off + vit->n - j - 1];
      else
        vit->met_brc[i] += inp[inp_off + vit->n - j - 1];
      
      tmp >>= 1;
    }
  }
}

static int
update_state_metric(janus_viterbi_t vit, unsigned* tbk_idx)
{
  // Next state.
  int sta_nxt = 0;
  // Current output.
  int out_cur = 0;
  unsigned renorm = C_MAX_METRIC;
  int sta_min = 0;

  unsigned i = 0;
  for (i = 0; i < vit->trellis->state_count; ++i)
    vit->met_tmp[i] = C_MAX_METRIC;
  
  for (i = 0; i < vit->trellis->state_count; ++i) 
  {
    unsigned j = 0;
    for (j = 0; j < (1U << vit->k); ++j) 
    {
      sta_nxt = vit->trellis->next_states[j * vit->trellis->state_count + i];
      out_cur = vit->trellis->outputs[j * vit->trellis->state_count + i];
      
      if (vit->met_sta[i] + vit->met_brc[out_cur] < vit->met_tmp[sta_nxt]) 
      {
        vit->met_tmp[sta_nxt] = vit->met_sta[i] + vit->met_brc[out_cur];
        vit->tbk_sta[sta_nxt + (*tbk_idx * vit->trellis->state_count)] = i;
        vit->tbk_inp[sta_nxt + (*tbk_idx * vit->trellis->state_count)] = j;
        
        if (vit->met_tmp[sta_nxt] < renorm) 
          renorm = vit->met_tmp[sta_nxt];
      }
    }
  }
  
  for (i = 0; i < vit->trellis->state_count; ++i) 
  {
    vit->met_sta[i] = vit->met_tmp[i] - renorm;
    if (vit->met_sta[i] == 0) 
      sta_min = i;
  }

  return sta_min;
}

static int
traceback(janus_viterbi_t vit, janus_uint8_t* out, unsigned blk_size, unsigned blk_idx, unsigned sta_min, unsigned* tbk_idx)
{
  int tbwork = *tbk_idx;
  int input = 0;
  int tbk_last = tbwork;

  janus_uint8_t* blk_out = NULL;
  
  if (blk_idx >= vit->tbk_depth) 
  {
    unsigned i = 0;

    for (i = 0; i < vit->tbk_depth + 1; ++i) 
    {
      input = (int)(vit->tbk_inp[sta_min + (tbwork * vit->trellis->state_count)]);
      sta_min = (int)(vit->tbk_sta[sta_min + (tbwork * vit->trellis->state_count)]);
      tbwork = (tbwork > 0) ? tbwork - 1 : vit->tbk_depth;
    }
    
    blk_out = out + (((blk_idx - vit->tbk_depth) % blk_size) * vit->k);
    for (i = 0; i < vit->k; ++i) 
    {
      blk_out[vit->k - 1 - i] = input & 1;
      input >>= 1;
    }
  }
  
  *tbk_idx = (*tbk_idx < vit->tbk_depth) ? *tbk_idx + 1 : 0;
    
  return tbk_last;
}

janus_viterbi_t
janus_viterbi_new(janus_trellis_t trellis, unsigned sdc_bits, unsigned tbk_depth)
{
  janus_viterbi_t vit = (janus_viterbi_t)calloc(1, sizeof(struct janus_viterbi));

  vit->trellis = trellis;
  vit->k = (unsigned)(JANUS_LOG2((janus_real_t)trellis->inp_sym_count));
  vit->n = (unsigned)(JANUS_LOG2((janus_real_t)trellis->out_sym_count));
  
  vit->sdc_bits = sdc_bits;

  vit->met_brc_size = 1 << vit->n;
  vit->met_brc = JANUS_UTILS_MEMORY_NEW_ZERO(unsigned, vit->met_brc_size);
  vit->met_tmp_size = trellis->state_count;
  vit->met_tmp = JANUS_UTILS_MEMORY_NEW_ZERO(unsigned, vit->met_tmp_size);
  vit->met_sta_size = trellis->state_count;
  vit->met_sta = JANUS_UTILS_MEMORY_NEW_ZERO(unsigned, vit->met_sta_size);

  vit->tbk_depth = 0;
  vit->tbk_inp = 0;
  vit->tbk_sta = 0;
  
  reset_traceback(vit, tbk_depth);
  reset_metric(vit);
  
  return vit;  
}

void
janus_viterbi_free(janus_viterbi_t vit)
{
  free(vit->met_brc);
  free(vit->met_tmp);
  free(vit->met_sta);
  free(vit->tbk_inp);
  free(vit->tbk_sta);
  free(vit);
}

void
janus_viterbi_execute(janus_viterbi_t vit, const janus_uint8_t* inp, unsigned inp_len, janus_uint8_t* out, unsigned tbk_depth)
{
  // Block size.
  unsigned blk_size = inp_len / vit->n;
  // Traceback index.
  unsigned tbk_idx = 0;
  // Minimum state.
  int sta_min = 0;

  int tbk_last = 0;
  unsigned i = 0; 
  int out_offs = 0;
  int input = 0;

  reset_traceback(vit, tbk_depth);
  reset_metric(vit);

  for (i = 0; i < blk_size; ++i)
  {
    branch_metric(vit, inp, i);
    sta_min = update_state_metric(vit, &tbk_idx);
    tbk_last = traceback(vit, out, blk_size, i, sta_min, &tbk_idx);
  }

  for (i = 0; i < vit->tbk_depth; ++i)
  {
    unsigned j = 0;

    out_offs = (blk_size - i - 1) * vit->k;

    input = (int)(vit->tbk_inp[sta_min + (tbk_last * vit->trellis->state_count)]);

    for (j = 0; j < vit->k; ++j)
    {
      out[out_offs + vit->k - 1 - j] = input & 1;
      input >>= 1;
    }

    sta_min = (int)(vit->tbk_sta[sta_min + (tbk_last * vit->trellis->state_count)]);
    tbk_last = (tbk_last > 0) ? tbk_last - 1 : vit->tbk_depth;
  }
}
