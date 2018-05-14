/******************************************************************************
 * Copyright (c) 2018 Texas Instruments Incorporated - http://www.ti.com/
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

//! @file imgutil.h

#pragma once
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
using namespace cv;

namespace tidl {
namespace imgutil {

//! PreProcImage - preprocess image data into the value ranges that
//! the DL network expects
//! @param image        Input image data (OpenCV data structure)
//! @param ptr          Output buffer that TI DL takes as input
//! @param roi          Number of Region-Of-Interests
//! @param n            Number of channels
//! @param width        Input image width
//! @param height       Input image height
//! @param pitch        Output buffer (ptr) pitch for each line
//! @param chOffset     Output buffer (ptr) offset for each channel
//! @param frameCount   Number of frames in input image data
//! @param preProcType  Preprocessing type, specified in network config
bool PreProcImage(Mat& image, char *ptr, int16_t roi, int16_t n,
                  int16_t width, int16_t height, int16_t pitch,
                  int32_t chOffset, int32_t frameCount, int32_t preProcType);

} // namesapce tidl::imgutil
} // namespace tidl
