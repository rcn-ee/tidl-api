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

//! @file execution_object_pipeline.h

#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <cassert>

#include "executor.h"
#include "execution_object_internal.h"
#include "execution_object.h"

namespace tidl {

/*! @class ExecutionObjectPipeline
    @brief Manages the pipelined execution using multiple ExecutionObjects.
    Each executor runs one layersGroup of the network.  ExecutionObjects
    must run consecutive layersGroups to form a pipelined execution.
*/
class ExecutionObjectPipeline : public ExecutionObjectInternalInterface
{
    public:
        //! @brief Create an ExecutionObjectPipeline object.
        //!
        //! The ExecutionObjectPipeline will take the provided ExecutionObjects
        //! to create an execution pipeline.  E.g.
        //! @code
        //!   Configuration config("path to configuration file");
        //!   DeviceIds ids = {DeviceId::ID0, DeviceId::ID1};
        //!   Executor exe_eve(DeviceType::EVE, ids, config, 1);
        //!   Executor exe_dsp(DeviceType::DSP, ids, config, 2);
        //!   ExecutionObjectPipeline ep0({exe_eve[0], exe_dsp[0]});
        //!   ExecutionObjectPipeline ep1({exe_eve[1], exe_dsp[1]});
        //! @endcode
        //!
        //! @param eos DSP or EVE ExecutionObjects forming a pipeline
        ExecutionObjectPipeline(std::vector<ExecutionObject*> eos);

        //! @brief Tear down an ExecutionObjectPipeline and free used resources
        ~ExecutionObjectPipeline();

        //! Specify the input and output buffers used by the EOP
        //! @param in buffer used for input.
        //! @param out buffer used for output.
        void SetInputOutputBuffer (const ArgInfo& in,
                                   const ArgInfo& out) override;

        //! Returns a pointer to the input buffer
        char* GetInputBufferPtr() const override;

        //! Returns size of the input buffer
        size_t GetInputBufferSizeInBytes() const override;

        //! Returns a pointer to the output buffer
        char* GetOutputBufferPtr() const override;

        //! Returns the number of bytes written to the output buffer
        size_t GetOutputBufferSizeInBytes() const override;

        //! @brief Set the frame index of the frame currently processed by the
        //! ExecutionObjectPipeline. Used for trace/debug messages
        //! @param idx index of the frame
        void SetFrameIndex(int idx) override;

        //! Returns the index of a frame being processed (set by SetFrameIndex)
        int  GetFrameIndex() const override;

        //! @brief Start processing a frame. The call is asynchronous and
        //! returns immediately. Use ProcessFrameWait() to wait
        bool ProcessFrameStartAsync() override;

        //! Wait for the executor pipeline to complete processing a frame
        //! @return false if ProcessFrameWait() was called
        //! without a corresponding call to
        //! ExecutionObjectPipeline::ProcessFrameStartAsync().
        bool ProcessFrameWait() override;

        //! @brief return the number of milliseconds taken *on the device* to
        //! execute the process call
        //! @return Number of milliseconds to process a frame on the device.
        float GetProcessTimeInMilliSeconds() const override;

        //! @brief return the number of milliseconds taken *on the host* to
        //! execute the process call
        //! @return Number of milliseconds to process a frame on the host.
        float GetHostProcessTimeInMilliSeconds() const override;

        //! Return the combined device names that this pipeline runs on
        const std::string& GetDeviceName() const override;

        //! Write the output buffer for each layer to a file
        //! \<filename_prefix>_<ID>_HxW.bin
        void WriteLayerOutputsToFile(const std::string& filename_prefix=
                                     "trace_dump_") const override;

        //! Returns a LayerOutput object corresponding to a layer.
        //! Caller is responsible for deleting the LayerOutput object.
        //! @see LayerOutput
        //! @param layer_index The layer index of the layer
        //! @param output_index The output index of the buffer for a given
        //!                     layer. Defaults to 0.
        const LayerOutput* GetOutputFromLayer(uint32_t layer_index,
                                       uint32_t output_index=0) const override;

        //! Get output buffers from all layers
        const LayerOutputs* GetOutputsFromAllLayers() const override;

        //! @private Used by runtime
        //! @brief callback function at the completion of each ExecutionObject,
        //! to chain the next ExectionObject for execution
        void RunAsyncNext();

        ExecutionObjectPipeline()                                     = delete;
        ExecutionObjectPipeline(const ExecutionObjectPipeline&)       = delete;
        ExecutionObjectPipeline& operator=(const ExecutionObjectPipeline&)
                                                                      = delete;

    private:
        class Impl;
        std::unique_ptr<Impl> pimpl_m;
};

} // namespace tidl
