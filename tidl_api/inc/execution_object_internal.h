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

/*! @file execution_object_internal.h */

#pragma once

namespace tidl {

class LayerOutput;

typedef std::vector<std::unique_ptr<const LayerOutput>> LayerOutputs;

/*! @cond HIDDEN_SYMBOLS
    @class ExecutionObjectInternalInterface
    @brief Internal interface for running the TIDL network on OpenCL devices
           Do not use this internal class directly.
           Please use ExecutionObject or ExecutionObejctPipeline instead.
*/
class ExecutionObjectInternalInterface
{
    public:
        virtual ~ExecutionObjectInternalInterface() {};

        //! Specify the input and output buffers used by the EO
        //! @param in buffer used for input.
        //! @param out buffer used for output.
        virtual void SetInputOutputBuffer(const ArgInfo& in,
                                          const ArgInfo& out) =0;

        //! Returns a pointer to the input buffer set via SetInputOutputBuffer
        virtual char* GetInputBufferPtr() const =0;

        //! Returns size of the input buffer
        virtual size_t GetInputBufferSizeInBytes() const =0;

        //! Returns a pointer to the output buffer
        virtual char* GetOutputBufferPtr() const =0;

        //! Returns size of the output buffer
        virtual size_t GetOutputBufferSizeInBytes() const =0;

        //! @brief Set the frame index of the frame currently processed by the
        //! ExecutionObject. Used for trace/debug messages
        //! @param idx index of the frame
        virtual void  SetFrameIndex(int idx) =0;

        //! Returns the index of a frame being processed (set by SetFrameIndex)
        virtual int   GetFrameIndex() const =0;

        //! @brief Start processing a frame. The call is asynchronous and returns
        //! immediately. Use ExecutionObject::ProcessFrameWait to wait
        virtual bool ProcessFrameStartAsync() =0;

        //! Wait for the execution object to complete processing a frame
        //! @return false if ExecutionObject::ProcessFrameWait was called
        //! without a corresponding call to
        //! ExecutionObject::ProcessFrameStartAsync.
        virtual bool ProcessFrameWait() =0;

        //! Returns the device name that the ExecutionObject runs on
        virtual const std::string& GetDeviceName() const =0;

        //! Write the output buffer for each layer to a file
        //! \<filename_prefix>_<ID>_HxW.bin
        virtual void WriteLayerOutputsToFile(const std::string& filename_prefix=
                                             "trace_dump_") const =0;

        //! Returns a LayerOutput object corresponding to a layer.
        //! Caller is responsible for deleting the LayerOutput object.
        //! @see LayerOutput
        //! @param layer_index The layer index of the layer
        //! @param output_index The output index of the buffer for a given
        //!                     layer. Defaults to 0.
        virtual const LayerOutput* GetOutputFromLayer(uint32_t layer_index,
                                             uint32_t output_index=0) const =0;

        //! Get output buffers from all layers
        virtual const LayerOutputs* GetOutputsFromAllLayers() const =0;
};
/*!  @endcond
*/

} // namespace tidl
