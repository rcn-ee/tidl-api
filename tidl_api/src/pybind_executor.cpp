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

/*! \file pybind_executor.cpp */

// Requires https://github.com/pybind/pybind11, branch v2.2
// References:
// 1. https://pybind11.readthedocs.io/en/stable/advanced/functions.html#return-value-policies
// 2. https://pybind11.readthedocs.io/en/stable/advanced/pycpp/numpy.html#buffer-protocol
// 3. https://www.python.org/dev/peps/pep-0008/

#include "pybind_common.h"

void AllocateMemory(const vector<ExecutionObject *>& eos);
void AllocateMemory(const vector<ExecutionObjectPipeline *>& eos);
void FreeMemory(const vector<ExecutionObject *>& eos);
void FreeMemory(const vector<ExecutionObjectPipeline *>& eos);

#define STRING(S)  XSTRING(S)
#define XSTRING(S) #S

PYBIND11_MAKE_OPAQUE(EO);

// Class, method names follow PEP8, Style Guide for Python Code (Reference #3)
PYBIND11_MODULE(tidl, m)
{
    m.doc() = std::string("TIDL API Python bindings ") + STRING(_BUILD_VER);
    m.attr("__version__") = STRING(_BUILD_VER);

    register_exception<Exception>(m, "TidlError");

    enum_<DeviceType>(m, "DeviceType")
        .value("DSP", DeviceType::DSP)
        .value("EVE", DeviceType::EVE);

    enum_<DeviceId>(m, "DeviceId")
        .value("ID0", DeviceId::ID0)
        .value("ID1", DeviceId::ID1)
        .value("ID2", DeviceId::ID2)
        .value("ID3", DeviceId::ID3);

    init_configuration(m);
    init_eo(m);
    init_eop(m);

    // For an explanation of return_value_policy, see Reference #1
	class_<Executor>(m, "Executor")
        .def(init<DeviceType, std::set<DeviceId>, Configuration, int>())

        .def("get_num_execution_objects", &Executor::GetNumExecutionObjects,
             "Returns number of ExecutionObjects created by this Executor")

        .def_static("get_num_devices", &Executor::GetNumDevices,
             "Returns number of devices of the specified type\n"
             "available for TI DL offload")

        .def_static("get_api_version", &Executor::GetAPIVersion)

        .def("at", &Executor::operator[], return_value_policy::reference,
            "Returns the ExecutionObject at the specified index");

    // Used to expose the EO's internal input and output buffers to the
    // python application using the Python buffer protocol (Reference #2)
    class_<ArgInfo>(m, "ArgInfo", buffer_protocol())
        .def_buffer([](ArgInfo& ai) -> buffer_info
                {
                    return buffer_info(
                            ai.ptr(),
                            sizeof(char),
                            pybind11::format_descriptor<char>::format(),
                            1,
                            {ai.size()},
                            {1}
                           );
                })
        .def("size", &ArgInfo::size, "Size of the buffer in bytes")
        .def("__repr__",
             [](const ArgInfo& ai)
             {
                std::stringstream ss;
                ss << "<ArgInfo: ptr= " << ai.ptr() << " size= " << ai.size()
                   << ">";
                return ss.str();
             });

    // Helper functions to allocate memory from Python
    m.def("allocate_memory",
          (void (*)(const vector<EO *>&)) &AllocateMemory,
          "Allocate input and output buffers for all ExecutionObjects");

    m.def("free_memory",
           (void (*)(const vector<EO *>&)) &FreeMemory,
          "Free input and output buffers of all ExecutionObjects");

    m.def("allocate_memory",
          (void (*)(const vector<EOP *>&)) &AllocateMemory,
          "Allocate input and output buffers for all ExecutionObjectPipelines");

    m.def("free_memory",
           (void (*)(const vector<EOP *>&)) &FreeMemory,
          "Free input and output buffers of all ExecutionObjectPipelines");

    m.def("enable_time_stamps",
           &EnableTimeStamps,
          "Enable timestamp generation for API events");
}

