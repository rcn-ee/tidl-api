/******************************************************************************
 * Copyright (c) 2017-2018  Texas Instruments Incorporated - http://www.ti.com/
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *       * Redistributions of source code must retain the above copyright
 *         notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *         notice, this list of conditions and the following disclaimer in the
 *         documentation and/or other materials provided with the distribution.
 *       * Neither the name of Texas Instruments Incorporated nor the
 *         names of its contributors may be used to endorse or promote products
 *         derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *   THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include "custom.h"
#include "dsp_c.h"

kernel 
void ocl_tidl_setup(global unsigned char*        createParams,
                    global unsigned char*        netParamsBuffer,
                    global unsigned char*        netParamsHeap,
                    global OCL_TIDL_SetupParams* setupParams)
{
    ocl_dsp_tidl_setup(createParams, netParamsBuffer, netParamsHeap, setupParams);
}

kernel 
void ocl_tidl_initialize(global unsigned char*            createParams,
                         global unsigned char*            netParamsBuffer,
                         global unsigned char*            externalMemoryHeapBase,
                         global OCL_TIDL_InitializeParams* initializeParams,
                         local  unsigned char*            l2HeapBase)
{
    // Set L1 cache to 16KB. TIDL requires 16KB of L1 scratch
    __cache_l1d_16k();

    ocl_dsp_tidl_initialize(createParams, netParamsBuffer, 
                            externalMemoryHeapBase, initializeParams, 
                            l2HeapBase);
}

kernel
void ocl_tidl_process(global OCL_TIDL_ProcessParams* processParams,
                      global unsigned char*          externalMemoryHeapBase,
                      global unsigned char*          traceBufferParams,
                      uint32_t                       contextIndex)
{
    ocl_dsp_tidl_process(processParams, externalMemoryHeapBase,
                         traceBufferParams, contextIndex);
}


kernel void ocl_tidl_cleanup()
{
    ocl_dsp_tidl_cleanup();
    __cache_l1d_all();
}
