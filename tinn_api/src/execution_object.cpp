/******************************************************************************
 * Copyright (c) 2017 Texas Instruments Incorporated - http://www.ti.com/
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

/*! \file execution_object.cpp */

#include "executor.h"
#include "execution_object.h"
#include "trace.h"
#include "ocl_device.h"
#include "parameters.h"
#include "configuration.h"
#include "common_defines.h"
#include <string.h>

using namespace tinn;

class ExecutionObject::Impl
{
    public:
        Impl(Device* d, uint8_t device_index,
             const ArgInfo& create_arg,
             const ArgInfo& param_heap_arg,
             size_t extmem_heap_size,
             uint32_t internal_input);
        ~Impl() {}

        bool RunAsync(CallType ct);
        bool Wait    (CallType ct);

        bool SetupProcessKernel(const ArgInfo& in, const ArgInfo& out);
        void HostWriteNetInput();
        void HostReadNetOutput();
        void ComputeInputOutputSizes();

        Device*                         device_m;
        std::unique_ptr<Kernel>         k_initialize_m;
        std::unique_ptr<Kernel>         k_process_m;
        std::unique_ptr<Kernel>         k_cleanup_m;

        up_malloc_ddr<char>             tidl_extmem_heap_m;
        up_malloc_ddr<OCL_TIDL_InitializeParams> shared_initialize_params_m;
        up_malloc_ddr<OCL_TIDL_ProcessParams>    shared_process_params_m;

        size_t                          in_size_m;
        size_t                          out_size_m;
        ArgInfo                         in_m;
        ArgInfo                         out_m;

        // Index of the OpenCL device/queue used by this EO
        uint8_t                         device_index_m;

        // Frame being processed by the EO
        int                             current_frame_idx_m;
};


ExecutionObject::ExecutionObject(Device* d,
                                 uint8_t device_index,
                                 const ArgInfo& create_arg,
                                 const ArgInfo& param_heap_arg,
                                 size_t extmem_heap_size,
                                 uint32_t internal_input)
{
    pimpl_m = std::unique_ptr<ExecutionObject::Impl>
              { new ExecutionObject::Impl(d, device_index,
                                          create_arg,
                                          param_heap_arg,
                                          extmem_heap_size,
                                          internal_input) };
}


ExecutionObject::Impl::Impl(Device* d,
                                 uint8_t device_index,
                                 const ArgInfo& create_arg,
                                 const ArgInfo& param_heap_arg,
                                 size_t extmem_heap_size,
                                 uint32_t internal_input):
    device_m(d),
    k_initialize_m(nullptr),
    k_process_m(nullptr),
    k_cleanup_m(nullptr),
    tidl_extmem_heap_m (nullptr, &__free_ddr),
    shared_initialize_params_m(nullptr, &__free_ddr),
    shared_process_params_m(nullptr, &__free_ddr),
    in_size_m(0),
    out_size_m(0),
    in_m(nullptr, 0),
    out_m(nullptr, 0),
    device_index_m(device_index),
    current_frame_idx_m(0)
{
    // Allocate a heap for TI DL to use on the device
    tidl_extmem_heap_m.reset(malloc_ddr<char>(extmem_heap_size));

    // Create a kernel for cleanup
    KernelArgs cleanup_args;
    k_cleanup_m.reset(new Kernel(device_m,
                                 STRING(CLEANUP_KERNEL),
                                 cleanup_args, device_index_m));

    // Set up parameter struct for the initialize kernel
    shared_initialize_params_m.reset(malloc_ddr<OCL_TIDL_InitializeParams>());
    memset(shared_initialize_params_m.get(), 0,
           sizeof(OCL_TIDL_InitializeParams));

    shared_initialize_params_m->tidlHeapSize = extmem_heap_size;
    shared_initialize_params_m->l2HeapSize   = tinn::internal::DMEM1_SIZE;
    shared_initialize_params_m->l1HeapSize   = tinn::internal::DMEM0_SIZE;
    shared_initialize_params_m->enableTrace  = OCL_TIDL_TRACE_OFF;
    shared_initialize_params_m->enableInternalInput = internal_input;

    // Setup kernel arguments for initialize
    KernelArgs args = { create_arg,
                        param_heap_arg,
                        ArgInfo(tidl_extmem_heap_m.get(),
                                extmem_heap_size),
                        ArgInfo(shared_initialize_params_m.get(),
                                sizeof(OCL_TIDL_InitializeParams)),
                        device_m->type() == CL_DEVICE_TYPE_ACCELERATOR ?
                            ArgInfo(nullptr, tinn::internal::DMEM1_SIZE):
                            ArgInfo(nullptr, 4)                       };

    k_initialize_m.reset(new Kernel(device_m,
                                    STRING(INIT_KERNEL), args, device_index_m));
}

// Pointer to implementation idiom: https://herbsutter.com/gotw/_100/:
// Both unique_ptr and shared_ptr can be instantiated with an incomplete type
// unique_ptr's destructor requires a complete type in order to invoke delete
ExecutionObject::~ExecutionObject() = default;

char* ExecutionObject::GetInputBufferPtr() const
{
    return static_cast<char *>(pimpl_m->in_m.ptr());
}

size_t ExecutionObject::GetInputBufferSizeInBytes() const
{
    if (pimpl_m->in_m.ptr() == nullptr)  return pimpl_m->in_size_m;
    else                                 return pimpl_m->in_m.size();
}

char* ExecutionObject::GetOutputBufferPtr() const
{
    return static_cast<char *>(pimpl_m->out_m.ptr());
}

size_t ExecutionObject::GetOutputBufferSizeInBytes() const
{
    if (pimpl_m->out_m.ptr() == nullptr)  return pimpl_m->out_size_m;
    else           return pimpl_m->shared_process_params_m.get()->bytesWritten;
}

void  ExecutionObject::SetFrameIndex(int idx)
{
    pimpl_m->current_frame_idx_m = idx;
}

int ExecutionObject::GetFrameIndex() const
{
    return pimpl_m->current_frame_idx_m;
}

void ExecutionObject::SetInputOutputBuffer(const ArgInfo& in, const ArgInfo& out)
{
    pimpl_m->SetupProcessKernel(in, out);
}

bool ExecutionObject::ProcessFrameStartAsync()
{
    return pimpl_m->RunAsync(ExecutionObject::CallType::PROCESS);
}

bool ExecutionObject::ProcessFrameWait()
{
    return pimpl_m->Wait(ExecutionObject::CallType::PROCESS);
}

bool ExecutionObject::RunAsync (CallType ct)
{
    return pimpl_m->RunAsync(ct);
}

bool ExecutionObject::Wait (CallType ct)
{
    return pimpl_m->Wait(ct);
}

uint64_t ExecutionObject::GetProcessCycles() const
{
    uint8_t factor = 1;

    // ARP32 running at half frequency of VCOP, multiply by 2 for VCOP cycles
    if (pimpl_m->device_m->type() == CL_DEVICE_TYPE_CUSTOM)
        factor = 2;

    return pimpl_m->shared_process_params_m.get()->cycles * factor;
}

float ExecutionObject::GetProcessTimeInMilliSeconds() const
{
    float frequency = pimpl_m->device_m->GetFrequencyInMhz() * 1000000;
    return ((float)GetProcessCycles())/frequency * 1000;
}

//
// Create a kernel to call the "process" function
//
bool
ExecutionObject::Impl::SetupProcessKernel(const ArgInfo& in, const ArgInfo& out)
{
    in_m = in;
    out_m = out;

    shared_process_params_m.reset(malloc_ddr<OCL_TIDL_ProcessParams>());
    shared_process_params_m->enableTrace = OCL_TIDL_TRACE_OFF;
    shared_process_params_m->enableInternalInput = 
                               shared_initialize_params_m->enableInternalInput;
    shared_process_params_m->cycles = 0;

    if (shared_process_params_m->enableInternalInput == 0)
        assert(in.ptr() != nullptr && in.size() > 0);

    KernelArgs args = { ArgInfo(shared_process_params_m.get(),
                                sizeof(OCL_TIDL_ProcessParams)),
                        in,
                        out,
                        ArgInfo(tidl_extmem_heap_m.get(),
                                shared_initialize_params_m->tidlHeapSize)
                      };

    k_process_m.reset(new Kernel(device_m,
                                 STRING(PROCESS_KERNEL), args, device_index_m));

    return true;
}


static size_t readDataS8(const char *readPtr, char *ptr, int roi, int n,
                         int width, int height, int pitch,
                         int chOffset)
{
    if (!readPtr)  return 0;

    for(int i2 = 0; i2 < roi; i2++)
        for(int i0 = 0; i0 < n; i0++)
            for(int i1 = 0; i1 < height; i1++)
                memcpy(&ptr[i2*n*chOffset + i0*chOffset + i1*pitch],
                       &readPtr[i2*n*width*height + i0*width*height+ i1*width],
                       width);

    return width*height*n*roi;
}

static size_t writeDataS8(char *writePtr, const char *ptr, int n, int width,
                          int height, int pitch, int chOffset)
{
    if (!writePtr)  return 0;

    for(int i0 = 0; i0 < n; i0++)
        for(int i1 = 0; i1 < height; i1++)
            memcpy(&writePtr[i0*width*height + i1*width],
                   &ptr[i0*chOffset + i1*pitch],
                   width);

    return width*height*n;
}

void ExecutionObject::Impl::HostWriteNetInput()
{
    char* readPtr  = (char *) in_m.ptr();
    PipeInfo *pipe = in_m.GetPipe();

    for (unsigned int i = 0; i < shared_initialize_params_m->numInBufs; i++)
    {
        OCL_TIDL_BufParams *inBuf = &shared_initialize_params_m->inBufs[i];

        if (shared_process_params_m->enableInternalInput == 0)
        {
            readPtr += readDataS8(
                readPtr,
                (char *) tidl_extmem_heap_m.get() + inBuf->bufPlaneBufOffset
                    + inBuf->bufPlaneWidth * OCL_TIDL_MAX_PAD_SIZE
                    + OCL_TIDL_MAX_PAD_SIZE,
                inBuf->numROIs,
                inBuf->numChannels,
                inBuf->ROIWidth,
                inBuf->ROIHeight,
                inBuf->bufPlaneWidth,
                ((inBuf->bufPlaneWidth * inBuf->bufPlaneHeight) /
                 inBuf->numChannels));
        }
        else
        {
            shared_process_params_m->inBufAddr[i] = pipe->bufAddr_m[i];
        }

        shared_process_params_m->inDataQ[i]   = pipe->dataQ_m[i];
    }
}

void ExecutionObject::Impl::HostReadNetOutput()
{
    char* writePtr = (char *) out_m.ptr();
    PipeInfo *pipe = out_m.GetPipe();

    for (unsigned int i = 0; i < shared_initialize_params_m->numOutBufs; i++)
    {
        OCL_TIDL_BufParams *outBuf = &shared_initialize_params_m->outBufs[i];
        if (writePtr != nullptr)
        {
            writePtr += writeDataS8(
                writePtr,
                (char *) tidl_extmem_heap_m.get() + outBuf->bufPlaneBufOffset
                    + outBuf->bufPlaneWidth * OCL_TIDL_MAX_PAD_SIZE
                    + OCL_TIDL_MAX_PAD_SIZE,
                outBuf->numChannels,
                outBuf->ROIWidth,
                outBuf->ROIHeight,
                outBuf->bufPlaneWidth,
                ((outBuf->bufPlaneWidth * outBuf->bufPlaneHeight)/
                 outBuf->numChannels));
        }

        pipe->dataQ_m[i]   = shared_process_params_m->outDataQ[i];
        pipe->bufAddr_m[i] = shared_initialize_params_m->bufAddrBase
                           + outBuf->bufPlaneBufOffset;
    }
    shared_process_params_m->bytesWritten = writePtr - (char *) out_m.ptr();
}

void ExecutionObject::Impl::ComputeInputOutputSizes()
{
    if (shared_initialize_params_m->errorCode != OCL_TIDL_SUCCESS)  return;

    if (shared_initialize_params_m->numInBufs > OCL_TIDL_MAX_IN_BUFS ||
        shared_initialize_params_m->numOutBufs > OCL_TIDL_MAX_OUT_BUFS)
    {
        std::cout << "Num input/output bufs ("
                  << shared_initialize_params_m->numInBufs << ", "
                  << shared_initialize_params_m->numOutBufs
                  << ") exceeded limit!" << std::endl;
        shared_initialize_params_m->errorCode = OCL_TIDL_INIT_FAIL;
        return;
    }

    in_size_m  = 0;
    out_size_m = 0;
    for (unsigned int i = 0; i < shared_initialize_params_m->numInBufs; i++)
    {
        OCL_TIDL_BufParams *inBuf = &shared_initialize_params_m->inBufs[i];
        in_size_m += inBuf->numROIs * inBuf->numChannels * inBuf->ROIWidth *
                     inBuf->ROIHeight;
    }
    for (unsigned int i = 0; i < shared_initialize_params_m->numOutBufs; i++)
    {
        OCL_TIDL_BufParams *outBuf = &shared_initialize_params_m->outBufs[i];
        out_size_m += outBuf->numChannels * outBuf->ROIWidth *outBuf->ROIHeight;
    }
}


bool ExecutionObject::Impl::RunAsync(CallType ct)
{
    switch (ct)
    {
        case CallType::INIT:
        {
            k_initialize_m->RunAsync();
            break;
        }
        case CallType::PROCESS:
        {
            shared_process_params_m->frameIdx = current_frame_idx_m;
            shared_process_params_m->bytesWritten = 0;
            HostWriteNetInput();
            k_process_m->RunAsync();
            break;
        }
        case CallType::CLEANUP:
        {
            k_cleanup_m->RunAsync();
            break;
        }
        default:
            return false;
    }

    return true;
}

bool ExecutionObject::Impl::Wait(CallType ct)
{
    switch (ct)
    {
        case CallType::INIT:
        {
            bool has_work = k_initialize_m->Wait();

            if (has_work)
            {
                ComputeInputOutputSizes();
                if (shared_initialize_params_m->errorCode != OCL_TIDL_SUCCESS)
                    throw Exception(shared_initialize_params_m->errorCode,
                                    __FILE__, __FUNCTION__, __LINE__);
            }
            return has_work;
        }
        case CallType::PROCESS:
        {
            bool has_work = k_process_m->Wait();
            if (has_work)
            {
                if (shared_process_params_m->errorCode != OCL_TIDL_SUCCESS)
                    throw Exception(shared_process_params_m->errorCode,
                                    __FILE__, __FUNCTION__, __LINE__);
                HostReadNetOutput();
            }

            return has_work;
        }
        case CallType::CLEANUP:
        {
            return k_cleanup_m->Wait();
            break;
        }
        default:
            return false;
    }

    return false;
}
