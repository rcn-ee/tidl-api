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

#pragma once

#include <stdint.h>
#include <vector>

namespace tidl {

/* @class SubgraphDataConv
   @brief Handles data conversion at subgraph boundaries
          At calibration time, consume either external input or external
          output tensors, determine sign and scaling factor.
          At inference time, use sign and scaling factor to perform data
          conversion between TIDL tensors and external tensors

   Example use for EstScaleQuant:
     SubgraphDataConv conv({}, {}, {}, {1,3,64,64,1,3,28,28});
     conv.EstScaleQuant(in);
     WriteQuantizationParams(conv.GetIsSigned(), conv.getScaleQ());
     conv.ScaleQuant(in, out);

   Example use for EstScaleDequant:
     SubgraphDataConv conv({}, {}, {}, {1,3,64,64,1,3,28,28});
     conv.EstScaleDeQuant(out);
     WriteDeQuantizationParams(conv.GetIsSigned(), conv.getScaleQ());

   Example use for ScaleQuant:
     // one time setup
     ... Parse json file for is_signed, scaleQ, is_NCHW, dims ...
     SubgraphDataConv conv(is_signed, scaleQ, is_NCHW, dims);

     // per inference
     out = eop.GetInputBufferPtr();
     conv.ScaleQuant(in, out);
     eop.ProcessFrameStartAsync();

   Example use for ScaleDeQuant:
     // one time setup
     ... Parse json file for is_signed, scaleQ, is_NCHW, dims ...
     SubgraphDataConv conv(is_signed, scaleQ, is_NCHW, dims);

     // per inference
     eop.ProcessFrameWait();
     in = eop.GetOutputBufferPtr();
     conv.ScaleDeQuant(in, out);
*/
class SubgraphDataConv
{
    public:
        //! @brief Creates a SubgraphDataConv.
        //! @param None
        SubgraphDataConv() {}

        SubgraphDataConv(const std::vector<bool>& is_signed,
                         const std::vector<float>& scaleQ,
                         const std::vector<bool>& is_NCHW,
                         const std::vector<int>& dims
                        ) : is_signed_m(is_signed), scaleQ_m(scaleQ),
                            is_NCHW_m(is_NCHW), dims_m(dims)
                        {}

        const std::vector<bool>&  GetIsSigned() { return is_signed_m; }
        const std::vector<float>& GetScaleQ()   { return scaleQ_m; }
        const std::vector<bool>&  GetIsNCHW()   { return is_NCHW_m; }

        //! @brief Estimate parameters for Quantization
        //! @param in vector of floating point external tensor data at input
        void EstScaleQuant(const std::vector<float*>& in);

        //! @brief Estimate paramters for DeQuantization
        //! @param out vector of floating point external tensor data at output
        void EstScaleDequant(const std::vector<float*>& out);

        //! @brief Quantizes floating point {in} to 8-bit Quantized {out}
        //!        and transposes buffer from NHWC to NCHW format (if needed),
        //!        results are put into out pointer consecutively, as expected
        //!        by TIDL
        //! @param in floating point vector input to quantize
        //! @param out 8-bit Quantized output (quantized from in)
        void ScaleQuant(const std::vector<float*>& in, uint8_t* out) const;

        //! @brief De-Quantizes 8-bit Quantized {in} to floating point {out}
        //!        and transposes buffer from NCHW to NHWC format (if needed),
        //!        the results are put into out vector, one vector per
        //!        tensor, as expected by external tensors
        //! @param in 8-bit Quantized input to De-Quantize
        //! @param out floating point output (De-Quantized from in)
        void ScaleDequant(const uint8_t *in, std::vector<float*>& out) const;

    private:
        //! if tensor needs to be evaluated as signed char
        std::vector<bool> is_signed_m;

        //! Q value for Quantization and Dequantization
        std::vector<float> scaleQ_m;

        //! the format of external tensors, NCHW or NHWC
        //! if data needs to be transposed between TIDL NCHW tensors and
        //! external tensors
        std::vector<bool> is_NCHW_m;

        //! flattened 4d dims of external tensors
        std::vector<int> dims_m;
};

}  // namespace tidl
