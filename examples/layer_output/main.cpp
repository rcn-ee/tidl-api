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
#include <signal.h>
#include <getopt.h>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>

#include "executor.h"
#include "execution_object.h"
#include "configuration.h"
#include "utils.h"

using namespace tidl;

bool RunConfiguration(const std::string& config_file, int num_devices,
                      DeviceType device_type);

static void ProcessTrace(const ExecutionObject* eo, const Configuration& c);

int main(int argc, char *argv[])
{
    // Catch ctrl-c to ensure a clean exit
    signal(SIGABRT, exit);
    signal(SIGTERM, exit);

    std::cout << "API Version: " << Executor::GetAPIVersion() << std::endl;

    // If there are no devices capable of offloading TIDL on the SoC, exit
    uint32_t num_eve = Executor::GetNumDevices(DeviceType::EVE);
    uint32_t num_dsp = Executor::GetNumDevices(DeviceType::DSP);
    if (num_eve == 0 && num_dsp == 0)
    {
        std::cout << "TI DL not supported on this SoC." << std::endl;
        return EXIT_SUCCESS;
    }

    // Configuration file with tracing enabled
    std::string config_file = "j11_v2_trace.txt";
    int         num_devices = 1;
    DeviceType  device_type = num_eve > 0 ? DeviceType::EVE :
                                            DeviceType::DSP;

    bool status = RunConfiguration(config_file, num_devices, device_type);

    if (!status)
    {
        std::cout << "tidl FAILED" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "tidl PASSED" << std::endl;
    return EXIT_SUCCESS;
}

bool RunConfiguration(const std::string& config_file, int num_devices,
                      DeviceType device_type)
{
    DeviceIds ids;
    for (int i = 0; i < num_devices; i++)
        ids.insert(static_cast<DeviceId>(i));

    // Read the TI DL configuration file
    Configuration c;
    if (!c.ReadFromFile(config_file))
        return false;

    // Open input files
    std::ifstream input(c.inData, std::ios::binary);
    assert (input.good());

    bool status = true;

    try
    {
        // Create a executor with the specified core type, number of cores
        // and configuration
        Executor E(device_type, ids, c);

        std::vector<ExecutionObject *> EOs;
        for (unsigned int i = 0; i < E.GetNumExecutionObjects(); i++)
            EOs.push_back(E[i]);

        int num_eos = EOs.size();

        // Allocate input and output buffers for each execution object
        for (auto eo : EOs)
        {
            size_t in_size  = eo->GetInputBufferSizeInBytes();
            size_t out_size = eo->GetOutputBufferSizeInBytes();
            ArgInfo in  = { ArgInfo(malloc(in_size),  in_size)};
            ArgInfo out = { ArgInfo(malloc(out_size), out_size)};
            eo->SetInputOutputBuffer(in, out);
        }

        // Process frames with available EOs in a pipelined manner
        // additional num_eos iterations to flush the pipeline (epilogue)
        for (int frame_idx = 0; frame_idx < c.numFrames + num_eos; frame_idx++)
        {
            ExecutionObject* eo = EOs[frame_idx % num_eos];

            // Wait for previous frame on the same eo to finish processing
            if (eo->ProcessFrameWait())
                ProcessTrace(eo, c);

            // Read a frame and start processing it with current eo
            if (ReadFrame(eo, frame_idx, c, input))
                eo->ProcessFrameStartAsync();
        }

        for (auto eo : EOs)
        {
            free(eo->GetInputBufferPtr());
            free(eo->GetOutputBufferPtr());
        }
    }
    catch (tidl::Exception &e)
    {
        std::cerr << e.what() << std::endl;
        status = false;
    }

    input.close();

    return status;
}


// APIs for accessing output buffers from individual layers
// 1. ExecutionObject::WriteLayerOutputsToFile
// 2. ExecutionObject::GetOutputsFromAllLayers
// 3. ExecutionObject::GetOutputFromLayer
void ProcessTrace(const ExecutionObject* eo, const Configuration& c)
{
    if (!c.enableOutputTrace)
    {
        std::cout << "Trace is not enabled. Set"
                     " Configuration::enableOutputTrace to true"
                  << std::endl;
        return;
    }

    // 1. Write the outputs from each layer to files
    // filename: trace_data_<layer_index>_<channels>_<width>_<height>.bin
    eo->WriteLayerOutputsToFile();

    // 2. Get all outputs from all layers and iterate through them
    const LayerOutputs* los = eo->GetOutputsFromAllLayers();
    if (!los) return;

    for (const std::unique_ptr<const LayerOutput> &lo : *los)
    {
        std::cout << "Layer index: " << lo->LayerIndex()
              << " Shape: " << lo->NumberOfChannels() << " x "
              << lo->Width() << " x " << lo->Height()
              << " Data ptr: " << static_cast<const void*>(lo->Data())
              << " Size in bytes: " << lo->Size()
              << std::endl;
    }

    // Call delete to free the memory used to store layer outputs
    delete los;

    // 3. Get the output from a single layer
    const LayerOutput* lo = eo->GetOutputFromLayer(1);
    if (!lo) return;

    std::cout << "Layer index: " << lo->LayerIndex()
          << " Shape: " << lo->NumberOfChannels() << " x "
          << lo->Width() << " x " << lo->Height()
          << " Data ptr: " << static_cast<const void*>(lo->Data())
          << " Size in bytes: " << lo->Size()
          << std::endl;

    delete lo;
}
