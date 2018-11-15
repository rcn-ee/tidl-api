/******************************************************************************
 * Copyright (c) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
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

#include <string.h>
#include <fstream>
#include <climits>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include "executor.h"
#include "execution_object.h"
#include "trace.h"
#include "ocl_device.h"
#include "parameters.h"
#include "common_defines.h"
#include "tidl_create_params.h"
#include "device_arginfo.h"
#include "util.h"

using namespace tidl;

class ExecutionObject::Impl
{
    public:
        Impl(Device* d, DeviceType t, uint8_t device_index,
             const DeviceArgInfo& create_arg,
             const DeviceArgInfo& param_heap_arg,
             const Configuration& configuration,
             int    layers_group_id);
        ~Impl() {}

        bool RunAsync(CallType ct, uint32_t context_idx);
        bool Wait    (CallType ct, uint32_t context_idx);
        bool AddCallback(CallType ct, void *user_data, uint32_t context_idx);

        uint64_t GetProcessCycles(uint32_t context_idx) const;
        int  GetLayersGroupId() const;
        void AcquireContext(uint32_t& context_idx);
        void ReleaseContext(uint32_t  context_idx);

        // Trace related
        void WriteLayerOutputsToFile (const std::string& filename_prefix) const;
        const LayerOutput* GetOutputFromLayer (uint32_t layer_index,
                                               uint32_t output_index) const;
        const LayerOutputs* GetOutputsFromAllLayers() const;


        Device*                         device_m;
        DeviceType                      device_type_m;

        // Index of the OpenCL device/queue used by this EO
        uint8_t                         device_index_m;
        std::string                     device_name_m;

        up_malloc_ddr<char>                      tidl_extmem_heap_m;
        up_malloc_ddr<OCL_TIDL_InitializeParams> shared_initialize_params_m;
        up_malloc_ddr<OCL_TIDL_ProcessParams>    shared_process_params_m;

        size_t                          in_size_m;
        size_t                          out_size_m;
        IODeviceArgInfo                 in_m[tidl::internal::NUM_CONTEXTS];
        IODeviceArgInfo                 out_m[tidl::internal::NUM_CONTEXTS];

        // Frame being processed by the EO
        int current_frame_idx_m[tidl::internal::NUM_CONTEXTS];

        // LayersGroupId being processed by the EO
        int layers_group_id_m;

        uint32_t                          num_network_layers_m;
        up_malloc_ddr<OCL_TIDL_BufParams> trace_buf_params_m;
        size_t                            trace_buf_params_sz_m;

    private:
        void SetupInitializeKernel(const DeviceArgInfo& create_arg,
                                   const DeviceArgInfo& param_heap_arg);
        void EnableOutputBufferTrace();
        void SetupProcessKernel();

        void HostWriteNetInput(uint32_t context_idx);
        void HostReadNetOutput(uint32_t context_idx);
        void ComputeInputOutputSizes();

        std::unique_ptr<Kernel>         k_initialize_m;
        std::unique_ptr<Kernel>         k_process_m;
        std::unique_ptr<Kernel>         k_cleanup_m;

        // Guarding sole access to input/output for one frame during execution
        // Encoding: context at bit index, bit value: 0 for idle, 1 for busy
        uint32_t                        idle_encoding_m;
        std::mutex                      mutex_access_m;
        std::condition_variable         cv_access_m;

        const Configuration             configuration_m;
};


ExecutionObject::ExecutionObject(Device*    d,
                                 DeviceType t,
                                 uint8_t    device_index,
                                 const      ArgInfo& create_arg,
                                 const      ArgInfo& param_heap_arg,
                                 const      Configuration& configuration,
                                 int        layers_group_id)
{
    TRACE::print("-> ExecutionObject::ExecutionObject()\n");

    DeviceArgInfo create_arg_d(create_arg, DeviceArgInfo::Kind::BUFFER);
    DeviceArgInfo param_heap_arg_d(param_heap_arg, DeviceArgInfo::Kind::BUFFER);

    pimpl_m = std::unique_ptr<ExecutionObject::Impl>
              { new ExecutionObject::Impl(d, t, device_index,
                                          create_arg_d,
                                          param_heap_arg_d,
                                          configuration,
                                          layers_group_id) };
    TRACE::print("<- ExecutionObject::ExecutionObject()\n");
}


ExecutionObject::Impl::Impl(Device* d, DeviceType t, uint8_t device_index,
                            const DeviceArgInfo& create_arg,
                            const DeviceArgInfo& param_heap_arg,
                            const Configuration& configuration,
                            int    layers_group_id):
    device_m(d),
    device_type_m(t),
    device_index_m(device_index),
    tidl_extmem_heap_m (nullptr, &__free_ddr),
    shared_initialize_params_m(nullptr, &__free_ddr),
    shared_process_params_m(nullptr, &__free_ddr),
    in_size_m(0),
    out_size_m(0),
    layers_group_id_m(layers_group_id),
    num_network_layers_m(0),
    trace_buf_params_m(nullptr, &__free_ddr),
    trace_buf_params_sz_m(0),
    k_initialize_m(nullptr),
    k_process_m(nullptr),
    k_cleanup_m(nullptr),
    idle_encoding_m(0),  // all contexts are idle
    configuration_m(configuration)
{
    device_name_m = device_m->GetDeviceName() + std::to_string(device_index_m);
    // Save number of layers in the network
    const TIDL_CreateParams* cp =
                static_cast<const TIDL_CreateParams *>(create_arg.ptr());
    num_network_layers_m = cp->net.numLayers;

    SetupInitializeKernel(create_arg, param_heap_arg);

    if (configuration_m.enableOutputTrace)
        EnableOutputBufferTrace();

    SetupProcessKernel();

    for (int i = 0; i < tidl::internal::NUM_CONTEXTS; i++)
        current_frame_idx_m[i] = 0;
}

// Pointer to implementation idiom: https://herbsutter.com/gotw/_100/:
// Both unique_ptr and shared_ptr can be instantiated with an incomplete type
// unique_ptr's destructor requires a complete type in order to invoke delete
ExecutionObject::~ExecutionObject() = default;

char* ExecutionObject::GetInputBufferPtr() const
{
    return static_cast<char *>(pimpl_m->in_m[0].GetArg().ptr());
}

size_t ExecutionObject::GetInputBufferSizeInBytes() const
{
    return pimpl_m->in_size_m;
}

char* ExecutionObject::GetOutputBufferPtr() const
{
    return static_cast<char *>(pimpl_m->out_m[0].GetArg().ptr());
}

size_t ExecutionObject::GetOutputBufferSizeInBytes() const
{
    return pimpl_m->out_size_m;
}

void  ExecutionObject::SetFrameIndex(int idx)
{
    pimpl_m->current_frame_idx_m[0] = idx;
}

int ExecutionObject::GetFrameIndex() const
{
    return pimpl_m->current_frame_idx_m[0];
}

void ExecutionObject::SetInputOutputBuffer(const ArgInfo& in,
                                           const ArgInfo& out)
{
    pimpl_m->in_m[0]  = IODeviceArgInfo(in);
    pimpl_m->out_m[0] = IODeviceArgInfo(out);
}

bool ExecutionObject::ProcessFrameStartAsync()
{
    return pimpl_m->RunAsync(ExecutionObject::CallType::PROCESS, 0);
}

bool ExecutionObject::ProcessFrameWait()
{
    return pimpl_m->Wait(ExecutionObject::CallType::PROCESS, 0);
}

bool ExecutionObject::RunAsync (CallType ct)
{
    return pimpl_m->RunAsync(ct, 0);
}

bool ExecutionObject::Wait (CallType ct)
{
    return pimpl_m->Wait(ct, 0);
}


bool ExecutionObject::AcquireAndRunContext(uint32_t& context_idx,
                                          int frame_idx,
                                          const IODeviceArgInfo& in,
                                          const IODeviceArgInfo& out)
{
    pimpl_m->AcquireContext(context_idx);

    pimpl_m->current_frame_idx_m[context_idx] = frame_idx;
    pimpl_m->in_m[context_idx]  = in;
    pimpl_m->out_m[context_idx] = out;

    return pimpl_m->RunAsync(ExecutionObject::CallType::PROCESS,
                                    context_idx);
}

bool ExecutionObject::WaitAndReleaseContext(uint32_t context_idx)
{
    TRACE::print("-> ExecutionObject::WaitAndReleaseContext(%d)\n",
                 context_idx);

    bool status = pimpl_m->Wait(ExecutionObject::CallType::PROCESS,
                                context_idx);
    pimpl_m->ReleaseContext(context_idx);

    return status;
}

bool ExecutionObject::AddCallback(CallType ct, void *user_data,
                                  uint32_t context_idx)
{
    return pimpl_m->AddCallback(ct, user_data, context_idx);
}

float ExecutionObject::GetProcessTimeInMilliSeconds() const
{
    float frequency = pimpl_m->device_m->GetFrequencyInMhz() * 1000000;
    return ((float)pimpl_m->GetProcessCycles(0)) / frequency * 1000;
}

void
ExecutionObject::WriteLayerOutputsToFile(const std::string& filename_prefix) const
{
    pimpl_m->WriteLayerOutputsToFile(filename_prefix);
}

const LayerOutput* ExecutionObject::GetOutputFromLayer(
                         uint32_t layer_index, uint32_t output_index) const
{
    return pimpl_m->GetOutputFromLayer(layer_index, output_index);
}

const LayerOutputs* ExecutionObject::GetOutputsFromAllLayers() const
{
    return pimpl_m->GetOutputsFromAllLayers();
}

int ExecutionObject::GetLayersGroupId() const
{
    return pimpl_m->layers_group_id_m;
}

const std::string& ExecutionObject::GetDeviceName() const
{
    return pimpl_m->device_name_m;
}


//
// Create a kernel to call the "initialize" function
//
void
ExecutionObject::Impl::SetupInitializeKernel(const DeviceArgInfo& create_arg,
                                             const DeviceArgInfo& param_heap_arg)
{
    // Allocate a heap for TI DL to use on the device
    tidl_extmem_heap_m.reset(
                         malloc_ddr<char>(configuration_m.NETWORK_HEAP_SIZE));

    // Create a kernel for cleanup
    KernelArgs cleanup_args;
    k_cleanup_m.reset(new Kernel(device_m,
                                 STRING(CLEANUP_KERNEL),
                                 cleanup_args, device_index_m));

    // Set up parameter struct for the initialize kernel
    shared_initialize_params_m.reset(malloc_ddr<OCL_TIDL_InitializeParams>());
    memset(shared_initialize_params_m.get(), 0,
           sizeof(OCL_TIDL_InitializeParams));

    shared_initialize_params_m->tidlHeapSize =configuration_m.NETWORK_HEAP_SIZE;
    shared_initialize_params_m->l2HeapSize   = tidl::internal::DMEM1_SIZE;
    shared_initialize_params_m->l1HeapSize   = tidl::internal::DMEM0_SIZE;
    shared_initialize_params_m->numContexts  = tidl::internal::NUM_CONTEXTS;

    // Set up execution trace specified in the configuration
    EnableExecutionTrace(configuration_m,
                         &shared_initialize_params_m->enableTrace);

    // Setup kernel arguments for initialize
    KernelArgs args = { create_arg,
                        param_heap_arg,
                        DeviceArgInfo(tidl_extmem_heap_m.get(),
                                      configuration_m.NETWORK_HEAP_SIZE,
                                      DeviceArgInfo::Kind::BUFFER),
                        DeviceArgInfo(shared_initialize_params_m.get(),
                                      sizeof(OCL_TIDL_InitializeParams),
                                      DeviceArgInfo::Kind::BUFFER),
                        device_m->type() == CL_DEVICE_TYPE_ACCELERATOR ?
                            DeviceArgInfo(nullptr, tidl::internal::DMEM1_SIZE,
                                          DeviceArgInfo::Kind::LOCAL):
                            DeviceArgInfo(nullptr, 4,
                                          DeviceArgInfo::Kind::LOCAL) };

    k_initialize_m.reset(new Kernel(device_m,
                                    STRING(INIT_KERNEL), args,
                                    device_index_m));
}

//
// Allocate an OpenCL buffer for TIDL layer output buffer metadata.
// The device will populate metadata for every buffer that is used as an
// output buffer by a layer.  This needs to be done before setting up
// process kernel.
//
void ExecutionObject::Impl::EnableOutputBufferTrace()
{
    trace_buf_params_sz_m = (sizeof(OCL_TIDL_BufParams)*
                             num_network_layers_m*
                             TIDL_NUM_OUT_BUFS);

    trace_buf_params_m.reset(malloc_ddr<OCL_TIDL_BufParams>
                             (trace_buf_params_sz_m));

    // Device will update bufferId if there is valid data for the entry
    OCL_TIDL_BufParams* bufferParams = trace_buf_params_m.get();
    for (uint32_t i = 0; i < num_network_layers_m; i++)
        for (int j = 0; j < TIDL_NUM_OUT_BUFS; j++)
        {
            OCL_TIDL_BufParams *bufP =
                                &bufferParams[i*TIDL_NUM_OUT_BUFS+j];
            bufP->bufferId = UINT_MAX;
        }
}

//
// Create a kernel to call the "process" function
//
void
ExecutionObject::Impl::SetupProcessKernel()
{
    shared_process_params_m.reset(malloc_ddr<OCL_TIDL_ProcessParams>(
               tidl::internal::NUM_CONTEXTS * sizeof(OCL_TIDL_ProcessParams)));

    // Set up execution trace specified in the configuration
    for (int i = 0; i < tidl::internal::NUM_CONTEXTS; i++)
    {
        OCL_TIDL_ProcessParams *p_params = shared_process_params_m.get() + i;
        EnableExecutionTrace(configuration_m, &p_params->enableTrace);
    }

    uint32_t context_idx = 0;
    KernelArgs args = { DeviceArgInfo(shared_process_params_m.get(),
                                      tidl::internal::NUM_CONTEXTS *
                                      sizeof(OCL_TIDL_ProcessParams),
                                      DeviceArgInfo::Kind::BUFFER),
                        DeviceArgInfo(tidl_extmem_heap_m.get(),
                                      shared_initialize_params_m->tidlHeapSize,
                                      DeviceArgInfo::Kind::BUFFER),
                        DeviceArgInfo(trace_buf_params_m.get(),
                                      trace_buf_params_sz_m,
                                      DeviceArgInfo::Kind::BUFFER),
                        DeviceArgInfo(&context_idx,
                                      sizeof(uint32_t),
                                      DeviceArgInfo::Kind::SCALAR)
                      };

    k_process_m.reset(new Kernel(device_m,
                                 STRING(PROCESS_KERNEL), args,
                                 device_index_m));
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

//
// Copy from host buffer to TIDL device buffer
//
void ExecutionObject::Impl::HostWriteNetInput(uint32_t context_idx)
{
    const char*     readPtr  = (const char *) in_m[context_idx].GetArg().ptr();
    const PipeInfo& pipe     = in_m[context_idx].GetPipe();
    OCL_TIDL_ProcessParams *p_params = shared_process_params_m.get()
                                       + context_idx;

    for (unsigned int i = 0; i < shared_initialize_params_m->numInBufs; i++)
    {
        OCL_TIDL_BufParams *inBuf = &shared_initialize_params_m->inBufs[i];
        char *inBufAddr = tidl_extmem_heap_m.get() + inBuf->bufPlaneBufOffset
                          + context_idx * inBuf->contextSize;

            readPtr += readDataS8(
                readPtr,
                (char *) inBufAddr
                    + inBuf->bufPlaneWidth * OCL_TIDL_MAX_PAD_SIZE
                    + OCL_TIDL_MAX_PAD_SIZE,
                inBuf->numROIs,
                inBuf->numChannels,
                inBuf->ROIWidth,
                inBuf->ROIHeight,
                inBuf->bufPlaneWidth,
                ((inBuf->bufPlaneWidth * inBuf->bufPlaneHeight) /
                 inBuf->numChannels));

        p_params->dataQ[i] = pipe.dataQ_m[i];
    }
}

//
// Copy from TIDL device buffer into host buffer
//
void ExecutionObject::Impl::HostReadNetOutput(uint32_t context_idx)
{
    char* writePtr = (char *) out_m[context_idx].GetArg().ptr();
    PipeInfo& pipe = out_m[context_idx].GetPipe();
    OCL_TIDL_ProcessParams *p_params = shared_process_params_m.get()
                                       + context_idx;

    for (unsigned int i = 0; i < shared_initialize_params_m->numOutBufs; i++)
    {
        OCL_TIDL_BufParams *outBuf = &shared_initialize_params_m->outBufs[i];
        char *outBufAddr = tidl_extmem_heap_m.get() + outBuf->bufPlaneBufOffset
                           + context_idx * outBuf->contextSize;
        if (writePtr != nullptr)
        {
            writePtr += writeDataS8(
                writePtr,
                (char *) outBufAddr
                    + outBuf->bufPlaneWidth * OCL_TIDL_MAX_PAD_SIZE
                    + OCL_TIDL_MAX_PAD_SIZE,
                outBuf->numChannels,
                outBuf->ROIWidth,
                outBuf->ROIHeight,
                outBuf->bufPlaneWidth,
                ((outBuf->bufPlaneWidth * outBuf->bufPlaneHeight)/
                 outBuf->numChannels));
        }

        pipe.dataQ_m[i]   = p_params->dataQ[i];
    }
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

bool ExecutionObject::Impl::RunAsync(CallType ct, uint32_t context_idx)
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
            RecordEvent(current_frame_idx_m[context_idx],
                        (layers_group_id_m == 1) ? TimeStamp::EO1_PFSA_START:
                                                   TimeStamp::EO2_PFSA_START,
                        static_cast<int>(device_type_m),
                        device_index_m);

            OCL_TIDL_ProcessParams *p_params = shared_process_params_m.get()
                                               + context_idx;
            p_params->frameIdx = current_frame_idx_m[context_idx];
            HostWriteNetInput(context_idx);
            {
                std::unique_lock<std::mutex> lock(mutex_access_m);
                k_process_m->UpdateScalarArg(3, sizeof(uint32_t), &context_idx);
                k_process_m->RunAsync(context_idx);
            }

            RecordEvent(current_frame_idx_m[context_idx],
                        (layers_group_id_m == 1) ?  TimeStamp::EO1_PFSA_END:
                                                    TimeStamp::EO2_PFSA_END);
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

bool ExecutionObject::Impl::Wait(CallType ct, uint32_t context_idx)
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
            RecordEvent(current_frame_idx_m[context_idx],
                        (layers_group_id_m == 1) ? TimeStamp::EO1_PFW_START:
                                                   TimeStamp::EO2_PFW_START);

            bool has_work = k_process_m->Wait(context_idx);
            if (has_work)
            {
                OCL_TIDL_ProcessParams *p_params = shared_process_params_m.get()
                                                   + context_idx;
                if (p_params->errorCode != OCL_TIDL_SUCCESS)
                    throw Exception(p_params->errorCode,
                                    __FILE__, __FUNCTION__, __LINE__);

                HostReadNetOutput(context_idx);

                RecordEvent(current_frame_idx_m[context_idx],
                            (layers_group_id_m == 1) ? TimeStamp::EO1_PFW_END:
                                                       TimeStamp::EO2_PFW_END);
            }
            else
            {
                // If there is no work, reset start event time
                ResetEvent(current_frame_idx_m[context_idx],
                           (layers_group_id_m == 1) ? TimeStamp::EO1_PFW_START:
                                                      TimeStamp::EO2_PFW_START);
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

bool ExecutionObject::Impl::AddCallback(CallType ct, void *user_data,
                                        uint32_t context_idx)
{
    switch (ct)
    {
        case CallType::PROCESS:
        {
            return k_process_m->AddCallback(user_data, context_idx);
            break;
        }
        default:
            return false;
    }

    return false;
}

uint64_t ExecutionObject::Impl::GetProcessCycles(uint32_t context_idx) const
{
    uint8_t factor = 1;

    // ARP32 running at half frequency of VCOP, multiply by 2 for VCOP cycles
    if (device_m->type() == CL_DEVICE_TYPE_CUSTOM)
        factor = 2;

    OCL_TIDL_ProcessParams *p_params = shared_process_params_m.get() +
                                       context_idx;
    return p_params->cycles * factor;
}

//
// Write the trace data to output files
//
void
ExecutionObject::Impl::WriteLayerOutputsToFile(const std::string& filename_prefix) const
{
    if (trace_buf_params_sz_m == 0)
        return;

    OCL_TIDL_BufParams* bufferParams = trace_buf_params_m.get();

    for (uint32_t i = 0; i < num_network_layers_m; i++)
        for (int j = 0; j < TIDL_NUM_OUT_BUFS; j++)
        {
            OCL_TIDL_BufParams* buf = &bufferParams[i*TIDL_NUM_OUT_BUFS+j];

            if (buf->bufferId == UINT_MAX)
                continue;

            size_t buffer_size = buf->numChannels * buf->ROIHeight *
                                 buf->ROIWidth;

            char *tmp = new char[buffer_size];

            if (tmp == nullptr)
                throw Exception("Out of memory, new failed",
                        __FILE__, __FUNCTION__, __LINE__);

            writeDataS8(
                tmp,
                (char *) tidl_extmem_heap_m.get() + buf->bufPlaneBufOffset
                + buf->bufPlaneWidth * OCL_TIDL_MAX_PAD_SIZE
                + OCL_TIDL_MAX_PAD_SIZE,
                buf->numChannels,
                buf->ROIWidth,
                buf->ROIHeight,
                buf->bufPlaneWidth,
                ((buf->bufPlaneWidth * buf->bufPlaneHeight)/
                 buf->numChannels));

            std::string filename(filename_prefix);
            filename += std::to_string(buf->bufferId) + "_";
            filename += std::to_string(buf->ROIWidth) + "x";
            filename += std::to_string(buf->ROIHeight) + ".bin";

            std::ofstream ofs;
            ofs.open(filename, std::ofstream::out);
            ofs.write(tmp, buffer_size);
            ofs.close();

            delete[] tmp;
        }
}


const LayerOutput* ExecutionObject::Impl::GetOutputFromLayer(
                            uint32_t layer_index, uint32_t output_index) const
{
    if (trace_buf_params_sz_m == 0)
        return nullptr;

    if (layer_index > num_network_layers_m || output_index > TIDL_NUM_OUT_BUFS)
        return nullptr;

    OCL_TIDL_BufParams* bufferParams = trace_buf_params_m.get();
    OCL_TIDL_BufParams* buf = &bufferParams[layer_index*TIDL_NUM_OUT_BUFS+
                                            output_index];

    if (buf->bufferId == UINT_MAX)
        return nullptr;

    size_t buffer_size = buf->numChannels * buf->ROIHeight *
                         buf->ROIWidth;

    char *data = new char[buffer_size];

    if (data == nullptr)
        throw Exception("Out of memory, new failed",
                __FILE__, __FUNCTION__, __LINE__);

    writeDataS8(data,
                (char *) tidl_extmem_heap_m.get() + buf->bufPlaneBufOffset
                + buf->bufPlaneWidth * OCL_TIDL_MAX_PAD_SIZE
                + OCL_TIDL_MAX_PAD_SIZE,
                buf->numChannels,
                buf->ROIWidth,
                buf->ROIHeight,
                buf->bufPlaneWidth,
                ((buf->bufPlaneWidth * buf->bufPlaneHeight)/
                 buf->numChannels));

    return new LayerOutput(layer_index, output_index, buf->bufferId,
                           buf->numROIs, buf->numChannels, buf->ROIHeight,
                           buf->ROIWidth, data);
}

const LayerOutputs* ExecutionObject::Impl::GetOutputsFromAllLayers() const
{
    LayerOutputs* result = new LayerOutputs;

    for (uint32_t i=0; i < num_network_layers_m; i++)
        for (int j=0; j < TIDL_NUM_OUT_BUFS; j++)
        {
            const LayerOutput* lo = GetOutputFromLayer(i, j);
            if (lo)
                result->push_back(std::unique_ptr<const LayerOutput>{ lo });
        }

    return result;
}

LayerOutput::LayerOutput(int layer_index, int output_index, int buffer_id,
                         int num_roi, int num_channels, size_t height,
                         size_t width, const char* data):
                        layer_index_m(layer_index), buffer_id_m(buffer_id),
                        num_roi_m(num_roi), num_channels_m(num_channels),
                        height_m(height), width_m(width), data_m(data)
{ }

LayerOutput::~LayerOutput()
{
    delete[] data_m;
}

void ExecutionObject::Impl::AcquireContext(uint32_t& context_idx)
{
    std::unique_lock<std::mutex> lock(mutex_access_m);
    cv_access_m.wait(lock, [this]{ return this->idle_encoding_m <
                                   (1 << tidl::internal::NUM_CONTEXTS) - 1; });

    for (uint32_t i = 0; i < tidl::internal::NUM_CONTEXTS; i++)
        if (((1 << i) & idle_encoding_m) == 0)
        {
            context_idx = i;
            break;
        }
    idle_encoding_m |= (1 << context_idx);  // mark the bit as busy
}

void ExecutionObject::Impl::ReleaseContext(uint32_t context_idx)
{
    {
        std::unique_lock<std::mutex> lock(mutex_access_m);
        idle_encoding_m &= (~(1 << context_idx));  // mark the bit as free
    }
    cv_access_m.notify_all();
}
