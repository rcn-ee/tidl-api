/******************************************************************************
 * Copyright (c) 2018-2018 Texas Instruments Incorporated - http://www.ti.com/
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

#include "pybind_common.h"

// Note: Overloaded methods must be disambiguated by casting them to
// function pointers. See
// https://pybind11.readthedocs.io/en/stable/classes.html#overloaded-methods
void init_eop(module &m)
{
    class_<EOP>(m, "ExecutionObjectPipeline")
        .def(init<std::vector<ExecutionObject *>>())

        .def("get_input_buffer",
              [](const EOP &eo)
              {
                 ArgInfo ai(eo.GetInputBufferPtr(),
                            eo.GetInputBufferSizeInBytes());
                 return ai;
              },
              "Exposes the input buffer in the ExecutionObjectPipeline as\n"
              "a Python object using the buffer protocol. This object can\n"
              "be specified as an argument to the readinto method")

        .def("get_output_buffer",
              [](const EOP &eo)
              {
                 ArgInfo ai(eo.GetOutputBufferPtr(),
                            eo.GetOutputBufferSizeInBytes());
                 return ai;
              },
              "Exposes the output buffer in the ExecutionObjectPipeline as\n"
              "a Python object using the buffer protocol. This object can\n"
              "be specified as an argument to the write method")

        .def("set_frame_index", &EOP::SetFrameIndex,
             "Set the frame index of the frame currently processed.\n"
             "Used for trace/debug messages")

        .def("get_frame_index", &EOP::GetFrameIndex)

        .def("process_frame_start_async", &EOP::ProcessFrameStartAsync,
             "Start processing a frame. The call is asynchronous and\n"
             "returns immediately")

        .def("process_frame_wait", &EOP::ProcessFrameWait,
             "Wait for the executor pipeline to complete processing a frame\n"
             "returns false if process_frame_wait() was called without a\n"
             " corresponding call to process_frame_start_async")

        .def("get_device_name", &EOP::GetDeviceName,
             "Returns the combined device names used by the pipeline");
}
