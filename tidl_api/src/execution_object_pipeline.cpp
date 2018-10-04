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

#include <assert.h>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include "device_arginfo.h"
#include "execution_object_pipeline.h"
#include "parameters.h"

using namespace tidl;

class ExecutionObjectPipeline::Impl
{
    public:
        Impl(std::vector<ExecutionObject*> &eos);
        ~Impl();

        void SetInputOutputBuffer(const ArgInfo &in, const ArgInfo &out);
        bool RunAsyncStart();
        bool RunAsyncNext();
        bool Wait();

        // Trace related
        void WriteLayerOutputsToFile(const std::string& filename_prefix) const;
        const LayerOutput* GetOutputFromLayer(uint32_t layer_index,
                                              uint32_t output_index) const;
        const LayerOutputs* GetOutputsFromAllLayers() const;

        //! for pipelined execution
        std::vector<ExecutionObject*> eos_m;
        std::vector<IODeviceArgInfo*> iobufs_m;
        std::vector<float> eo_device_time_m;
        std::vector<float> eo_host_time_m;

        std::string device_name_m;

        //! current frame index
        int frame_idx_m;

        //! current execution object index, and it context index
        uint32_t curr_eo_idx_m;
        uint32_t curr_eo_context_idx_m;

        // device and host time tracking: pipeline start to finish
        float device_time_m;
        float host_time_m;

    private:
        //! @brief Initialize ExecutionObjectPipeline with given
        //! ExecutionObjects: check consecutive layersGroup, allocate memory
        void Initialize();

        // flag, mutex and cond var for signaling completion and waiting
        bool has_work_m, is_processed_m;
        std::mutex mutex_m;
        std::condition_variable cv_m;

        // host time tracking: pipeline start to finish
        std::chrono::time_point<std::chrono::steady_clock> start_m;
};

ExecutionObjectPipeline::ExecutionObjectPipeline(
    std::vector<ExecutionObject*> eos)
{
    pimpl_m = std::unique_ptr<Impl> { new Impl(eos) };
}

ExecutionObjectPipeline::Impl::Impl(std::vector<ExecutionObject *> &eos) :
    eos_m(eos), has_work_m(false), is_processed_m(false)
{
    Initialize();
}

// Pointer to implementation idiom: https://herbsutter.com/gotw/_100/:
// Both unique_ptr and shared_ptr can be instantiated with an incomplete type
// unique_ptr's destructor requires a complete type in order to invoke delete
ExecutionObjectPipeline::~ExecutionObjectPipeline() = default;

char* ExecutionObjectPipeline::GetInputBufferPtr() const
{
    return static_cast<char *>(pimpl_m->iobufs_m.front()->GetArg().ptr());
}

uint32_t ExecutionObjectPipeline::GetNumExecutionObjects() const
{
    return pimpl_m->eos_m.size();
}

size_t ExecutionObjectPipeline::GetInputBufferSizeInBytes() const
{
    return pimpl_m->eos_m.front()->GetInputBufferSizeInBytes();
}

char* ExecutionObjectPipeline::GetOutputBufferPtr() const
{
    return static_cast<char *>(pimpl_m->iobufs_m.back()->GetArg().ptr());
}

size_t ExecutionObjectPipeline::GetOutputBufferSizeInBytes() const
{
    return pimpl_m->eos_m.back()->GetOutputBufferSizeInBytes();
}

void ExecutionObjectPipeline::SetInputOutputBuffer(const ArgInfo& in,
                                                   const ArgInfo& out)
{
    assert(in.ptr() != nullptr  && in.size() >= GetInputBufferSizeInBytes());
    assert(out.ptr() != nullptr && out.size() >= GetOutputBufferSizeInBytes());
    pimpl_m->SetInputOutputBuffer(in, out);
}

void ExecutionObjectPipeline::SetFrameIndex(int idx)
{
    pimpl_m->frame_idx_m = idx;
}

int ExecutionObjectPipeline::GetFrameIndex() const
{
    return pimpl_m->frame_idx_m;
}

bool ExecutionObjectPipeline::ProcessFrameStartAsync()
{
    assert(GetInputBufferPtr() != nullptr && GetOutputBufferPtr() != nullptr);
    bool st = pimpl_m->RunAsyncStart();
    if (st)
        st = pimpl_m->eos_m[0]->AddCallback(ExecutionObject::CallType::PROCESS,
                                         this, pimpl_m->curr_eo_context_idx_m);
    return st;
}

bool ExecutionObjectPipeline::ProcessFrameWait()
{
    return pimpl_m->Wait();
}

void CallbackWrapper(void *user_data)
{
    ((ExecutionObjectPipeline *) user_data)->RunAsyncNext();
}

void ExecutionObjectPipeline::RunAsyncNext()
{
    bool has_next = pimpl_m->RunAsyncNext();
    if (has_next)
        pimpl_m->eos_m[pimpl_m->curr_eo_idx_m]->AddCallback(
                                     ExecutionObject::CallType::PROCESS, this,
                                     pimpl_m->curr_eo_context_idx_m);
}

float ExecutionObjectPipeline::GetProcessTimeInMilliSeconds() const
{
    return pimpl_m->device_time_m;
}

float ExecutionObjectPipeline::GetHostProcessTimeInMilliSeconds() const
{
    return pimpl_m->host_time_m;
}

float ExecutionObjectPipeline::GetProcessTimeInMilliSeconds(
        uint32_t eo_index) const
{
    assert(eo_index < pimpl_m->eos_m.size());
    return pimpl_m->eo_device_time_m[eo_index];
}

float ExecutionObjectPipeline::GetHostProcessTimeInMilliSeconds(
        uint32_t eo_index) const
{
    assert(eo_index < pimpl_m->eos_m.size());
    return pimpl_m->eo_host_time_m[eo_index];
}

const std::string& ExecutionObjectPipeline::GetDeviceName() const
{
    return pimpl_m->device_name_m;
}

void
ExecutionObjectPipeline::WriteLayerOutputsToFile(
    const std::string& filename_prefix) const
{
    pimpl_m->WriteLayerOutputsToFile(filename_prefix);
}

const LayerOutput*
ExecutionObjectPipeline::GetOutputFromLayer(uint32_t layer_index,
    uint32_t output_index) const
{
    return pimpl_m->GetOutputFromLayer(layer_index, output_index);
}

const LayerOutputs*
ExecutionObjectPipeline::GetOutputsFromAllLayers() const
{
    return pimpl_m->GetOutputsFromAllLayers();
}


/// Impl methods start here


static
void* AllocateMem(size_t size)
{
    if (size == 0)  return nullptr;
    void *ptr = malloc(size);
    if (ptr == nullptr)
        throw Exception("Out of memory, ExecutionObjectPipeline malloc failed",
                        __FILE__, __FUNCTION__, __LINE__);
    return ptr;
}

void ExecutionObjectPipeline::Impl::Initialize()
{
    // Check consecutive layersGroups to form a pipeline
    int prev_group = 0;
    for (auto eo : eos_m)
    {
        int group = eo->GetLayersGroupId();
        if (prev_group != 0 && group != prev_group + 1)
            throw Exception(
                "Non-consecutive layersGroupIds in ExecutionObjectPipeline",
                __FILE__, __FUNCTION__, __LINE__);
        prev_group = group;
    }

    for (auto eo : eos_m)
        device_name_m += eo->GetDeviceName() + "+";
    device_name_m.resize(device_name_m.size() - 1);

    // Allocate input and output memory for EOs/layersGroups
    // Note that i-th EO's output buffer is the same as (i+1)-th EO's input
    // So, if n EOs, then (n+1) buffers: b EO b EO b EO b ... EO b
    // User must set the first input buffer and the last output buffer
    size_t size;
    ArgInfo in(nullptr, 0);
    iobufs_m.push_back(new IODeviceArgInfo(in));
    for (auto eo : eos_m)
    {
        if (eo != eos_m.back())
            size = eo->GetOutputBufferSizeInBytes();
        else
            size = 0;

        void *ptr = AllocateMem(size);
        ArgInfo out(ptr, size);
        iobufs_m.push_back(new IODeviceArgInfo(out));
    }

    // Record keeping for each EO's device time and host time
    // because EO could be shared by another EOP
    eo_device_time_m.resize(eos_m.size());
    eo_host_time_m.resize(eos_m.size());
}

ExecutionObjectPipeline::Impl::~Impl()
{
    int num_iobufs = iobufs_m.size();
    for (int i = 0; i < num_iobufs; i++)
    {
        if (! (i == 0 || i == num_iobufs-1))
            free(iobufs_m[i]->GetArg().ptr());
        delete iobufs_m[i];
    }
}

void ExecutionObjectPipeline::Impl::SetInputOutputBuffer(const ArgInfo &in,
                                                         const ArgInfo &out)
{
    delete iobufs_m.front();
    delete iobufs_m.back();
    iobufs_m.front() = new IODeviceArgInfo(in);
    iobufs_m.back()  = new IODeviceArgInfo(out);
}

bool ExecutionObjectPipeline::Impl::RunAsyncStart()
{
    has_work_m = true;
    is_processed_m = false;
    device_time_m = 0.0f;
    host_time_m = 0.0f;
    curr_eo_idx_m = 0;
    eos_m[0]->AcquireContext(curr_eo_context_idx_m);
    if (tidl::internal::NUM_CONTEXTS == 1)
        start_m = std::chrono::steady_clock::now();
    eos_m[0]->SetFrameIndex(frame_idx_m);
    eos_m[0]->SetInputOutputBuffer(iobufs_m[0], iobufs_m[1],
                                   curr_eo_context_idx_m);
    return eos_m[0]->ProcessFrameStartAsync(curr_eo_context_idx_m);
}

// returns true if we have more EOs to execute
bool ExecutionObjectPipeline::Impl::RunAsyncNext()
{
    eos_m[curr_eo_idx_m]->ProcessFrameWait(curr_eo_context_idx_m);
    // need to capture EO's device/host time before we release its lock
    eo_device_time_m[curr_eo_idx_m] = eos_m[curr_eo_idx_m]->
                           GetProcessTimeInMilliSeconds(curr_eo_context_idx_m);
    eo_host_time_m[curr_eo_idx_m]   = eos_m[curr_eo_idx_m]->
                       GetHostProcessTimeInMilliSeconds(curr_eo_context_idx_m);
    device_time_m += eo_device_time_m[curr_eo_idx_m];
    if (tidl::internal::NUM_CONTEXTS > 1)
        host_time_m += eo_host_time_m[curr_eo_idx_m];
    eos_m[curr_eo_idx_m]->ReleaseContext(curr_eo_context_idx_m);
    curr_eo_idx_m += 1;
    if (curr_eo_idx_m < eos_m.size())
    {
        eos_m[curr_eo_idx_m]->AcquireContext(curr_eo_context_idx_m);
        eos_m[curr_eo_idx_m]->SetFrameIndex(frame_idx_m);
        eos_m[curr_eo_idx_m]->SetInputOutputBuffer(iobufs_m[curr_eo_idx_m],
                          iobufs_m[curr_eo_idx_m+1], curr_eo_context_idx_m);
        eos_m[curr_eo_idx_m]->ProcessFrameStartAsync(curr_eo_context_idx_m);
        return true;
    }
    else
    {
        if (tidl::internal::NUM_CONTEXTS == 1)
        {
            std::chrono::duration<float> elapsed =
                                    std::chrono::steady_clock::now() - start_m;
            host_time_m = elapsed.count() * 1000;  // seconds to milliseconds
        }
        is_processed_m = true;
        cv_m.notify_all();
        return false;
    }
}

bool ExecutionObjectPipeline::Impl::Wait()
{
    if (! has_work_m)  return false;

    std::unique_lock<std::mutex> lock(mutex_m);
    cv_m.wait(lock, [this]{ return this->is_processed_m; });
    has_work_m = false;
    return true;
}

void
ExecutionObjectPipeline::Impl::WriteLayerOutputsToFile(
    const std::string& filename_prefix) const
{
    for (auto eo : eos_m)
        eo->WriteLayerOutputsToFile(filename_prefix);
}

const LayerOutput*
ExecutionObjectPipeline::Impl::GetOutputFromLayer(uint32_t layer_index,
    uint32_t output_index) const
{
    const LayerOutput* lo = nullptr;
    for (auto eo : eos_m)
    {
        lo = eo->GetOutputFromLayer(layer_index, output_index);
        if (lo != nullptr)  break;
    }
    return lo;
}

const LayerOutputs*
ExecutionObjectPipeline::Impl::GetOutputsFromAllLayers() const
{
    LayerOutputs *all = new LayerOutputs;
    for (auto eo : eos_m)
    {
        LayerOutputs *los = const_cast<LayerOutputs *>(
                                                eo->GetOutputsFromAllLayers());
        for (auto& lo : *los)
            all->push_back(std::unique_ptr<const LayerOutput>{ lo.release() });
        delete los;
    }
    return all;
}

