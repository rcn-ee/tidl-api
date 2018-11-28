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

void init_configuration(module &m)
{
    class_<Configuration>(m, "Configuration")
		.def(init<>())
        .def_readwrite("num_frames", &Configuration::numFrames,
            "Number of frames of input data. Can be 0 if data is\n"
            "not read from file")

        .def_readwrite("height", &Configuration::inHeight)

        .def_readwrite("width", &Configuration::inWidth)

        .def_readwrite("channels", &Configuration::inNumChannels)

        .def_readwrite("pre_proc_type", &Configuration::preProcType,
            "Pre-processing type applied to the input frame")

        .def_readwrite("run_full_net", &Configuration::runFullNet,
            "Force to run all layers, regardless of layersGroupId partitioning")

        .def_readwrite("network_binary", &Configuration::netBinFile,
            "Path to the TIDL network binary file")

        .def_readwrite("parameter_binary", &Configuration::paramsBinFile,
            "Path to the TIDL parameter binary file")

        .def_readwrite("network_heap_size", &Configuration::NETWORK_HEAP_SIZE,
            "Size of the device side network heap\n"
            "This heap is used for allocating memory required to"
            "run the network on the device. One per ExecutionObject.")

        .def_readwrite("param_heap_size", &Configuration::PARAM_HEAP_SIZE,
            "Size of the device side heap used for parameter data.\n"
            "The size depends on the size of the parameter binary file.\n"
            "There is one parameter heap for each instance of Executor\n")

        .def_readwrite("enable_api_trace", &Configuration::enableApiTrace,
            "Debug - Set to True to generate a trace of host/device\n"
            "function calls")

        .def_readwrite("enable_layer_dump", &Configuration::enableOutputTrace,
            "Debug - Enable dump of output buffers associated with each layer")

        .def_readwrite("show_heap_stats", &Configuration::showHeapStats,
            "Debug - Shows total size of PARAM and NETWORK heaps. Also \n"
            "shows bytes free after all allocations. Used to adjust heap sizes")

        .def_readwrite("layer_index_to_layer_group_id",
                       &Configuration::layerIndex2LayerGroupId,
            "Map of layer index to layer group id. Used to override \n"
            "layer group assigment for layers. Any layer not specified\n"
            "in this map will retain its existing mapping")

        .def_readwrite("in_data", &Configuration::inData)

        .def_readwrite("out_data", &Configuration::outData)

        .def("read_from_file", &Configuration::ReadFromFile,
             "Read a configuration from the specified file");
}
