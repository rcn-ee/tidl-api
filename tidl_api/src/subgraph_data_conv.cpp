/******************************************************************************
 * Copyright (c) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of Texas Instruments Incorporated nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *  THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "subgraph_data_conv.h"

using namespace tidl;


static inline uint8_t QuantizeValue(float v, float Q, int vmin, int vmax)
{
  // scale
  int32_t qv = (int32_t) (v * Q);
  // saturate
  qv = qv < vmin ? vmin : qv;
  qv = qv > vmax ? vmax : qv;
  return (uint8_t) qv;
}

static inline float DequantizeValue(uint8_t v, float Q_inv, bool S)
{
  // interpret sign
  int32_t sv = S ? ((int32_t)(int8_t) v) : ((int32_t) v);
  // scale
  return sv * Q_inv;
}

// Gets 1-d index for 4-d buffer[d][c][b][a]
static inline int GetIndex(int d, int c, int b, int a,
                           int D, int C, int B, int A)
{
    return a + A*(b + B*(c + C*d));
}


void SubgraphDataConv::EstScaleQuant(const std::vector<float*>& in)
{
  // TODO
}

void SubgraphDataConv::EstScaleDequant(const std::vector<float*>& out)
{
  // TODO
}


void
SubgraphDataConv::ScaleQuant(const std::vector<float*>& in, uint8_t* out)
const
{
  int offset = 0;
  for (uint32_t d = 0; d < is_NCHW_m.size(); d++)
  {
    float Q     = scaleQ_m[d];
    int   N     = dims_m[4 * d + 0];
    int   C     = dims_m[4 * d + 1];
    int   H     = dims_m[4 * d + 2];
    int   W     = dims_m[4 * d + 3];
    int   vmin  = is_signed_m[d] ? -128 : 0;
    int   vmax  = is_signed_m[d] ?  127 : 255;
    float *in_d = in[d];
    if (is_NCHW_m[d])  // no need to transpose external tensor
    {
      for (int i = 0; i < N * C * H * W; i++)
        out[offset + i] = QuantizeValue(in_d[i], Q, vmin, vmax);
    }
    else  // need to transpose external tensor
    {
      for (int n = 0; n < N; n++)
        for (int c = 0; c < C; c++)
          for (int h = 0; h < H; h++)
            for (int w = 0; w < W; w++)
            {
              int nchw = GetIndex(n, c, h, w, N, C, H, W);
              int nhwc = GetIndex(n, h, w, c, N, H, W, C);
              out[offset + nchw] = QuantizeValue(in_d[nhwc], Q, vmin, vmax);
            }
    }
    offset += N * C * H * W;
  }
}

void
SubgraphDataConv::ScaleDequant(const uint8_t *in, std::vector<float*>& out)
const
{
  int offset = 0;
  for (uint32_t d = 0; d < is_NCHW_m.size(); d++)
  {
    float Q      = scaleQ_m[d];
    float Q_inv  = 1.0f / Q;
    int   N      = dims_m[4 * d + 0];
    int   C      = dims_m[4 * d + 1];
    int   H      = dims_m[4 * d + 2];
    int   W      = dims_m[4 * d + 3];
    bool  S      = is_signed_m[d];
    float *out_d = out[d];
    if (is_NCHW_m[d])  // no need to transpose external tensor
    {
      for (int i = 0; i < N * C * H * W; i++)
        out_d[i] = DequantizeValue(in[offset + i], Q_inv, S);
    }
    else  // need to transpose external tensor
    {
      for (int n = 0; n < N; n++)
        for (int c = 0; c < C; c++)
          for (int h = 0; h < H; h++)
            for (int w = 0; w < W; w++)
            {
              int nchw = GetIndex(n, c, h, w, N, C, H, W);
              int nhwc = GetIndex(n, h, w, c, N, H, W, C);
              out_d[nhwc] = DequantizeValue(in[offset + nchw], Q_inv, S);
            }
    }
    offset += N * C * H * W;
  }
}

