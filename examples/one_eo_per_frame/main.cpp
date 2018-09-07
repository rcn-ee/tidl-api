/******************************************************************************
 * Copyright (c) 2017-2018  Texas Instruments Incorporated - http://www.ti.com/
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

//
// This example illustrates using a single EO to process a frame. To increase
// throughput, multiple EOs are used.
// For details, refer http://downloads.ti.com/mctools/esd/docs/tidl-api/
//
#include <signal.h>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>

#include "executor.h"
#include "execution_object.h"
#include "configuration.h"
#include "utils.h"

using namespace tidl;
using std::string;
using std::unique_ptr;
using std::vector;

bool Run(const string& config_file, int num_eve,int num_dsp,
         const char* ref_output);

Executor* CreateExecutor(DeviceType dt, int num, const Configuration& c);
void      CollectEOs(const Executor *e, vector<ExecutionObject *>& EOs);


int main(int argc, char *argv[])
{
    // Catch ctrl-c to ensure a clean exit
    signal(SIGABRT, exit);
    signal(SIGTERM, exit);

    // If there are no devices capable of offloading TIDL on the SoC, exit
    uint32_t num_eve = Executor::GetNumDevices(DeviceType::EVE);
    uint32_t num_dsp = Executor::GetNumDevices(DeviceType::DSP);
    if (num_eve == 0 && num_dsp == 0)
    {
        std::cout << "TI DL not supported on this SoC." << std::endl;
        return EXIT_SUCCESS;
    }

    string config_file ="../test/testvecs/config/infer/tidl_config_j11_v2.txt";
    string ref_file    ="../test/testvecs/reference/j11_v2_ref.bin";

    unique_ptr<const char> reference_output(ReadReferenceOutput(ref_file));

    bool status = Run(config_file, num_eve, num_dsp, reference_output.get());

    if (!status)
    {
        std::cout << "FAILED" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "PASSED" << std::endl;
    return EXIT_SUCCESS;
}

bool Run(const string& config_file, int num_eve, int num_dsp,
         const char* ref_output)
{
    Configuration c;
    if (!c.ReadFromFile(config_file))
        return false;

    // Heap sizes for this network determined using Configuration::showHeapStats
    c.PARAM_HEAP_SIZE   = (3 << 20); // 3MB
    c.NETWORK_HEAP_SIZE = (20 << 20); // 20MB

    c.numFrames = 16;

    // Open input file for reading
    std::ifstream input_data_file(c.inData, std::ios::binary);

    bool status = true;
    try
    {
        // Create Executors - use all the DSP and EVE cores available
        unique_ptr<Executor> e_dsp(CreateExecutor(DeviceType::DSP, num_dsp, c));
        unique_ptr<Executor> e_eve(CreateExecutor(DeviceType::EVE, num_eve, c));

        // Accumulate all the EOs from across the Executors
        vector<ExecutionObject *> EOs;
        CollectEOs(e_eve.get(), EOs);
        CollectEOs(e_dsp.get(), EOs);

        AllocateMemory(EOs);

        // Process frames with EOs in a pipelined manner
        // additional num_eos iterations to flush the pipeline (epilogue)
        int num_eos = EOs.size();
        for (int frame_idx = 0; frame_idx < c.numFrames + num_eos; frame_idx++)
        {
            ExecutionObject* eo = EOs[frame_idx % num_eos];

            // Wait for previous frame on the same eo to finish processing
            if (eo->ProcessFrameWait())
            {
                ReportTime(eo);
                if (frame_idx < num_eos && !CheckFrame(eo, ref_output))
                    status = false;
            }

            // Read a frame and start processing it with current eo
            if (ReadFrame(eo, frame_idx, c, input_data_file))
                eo->ProcessFrameStartAsync();
        }

        FreeMemory(EOs);

    }
    catch (tidl::Exception &e)
    {
        std::cerr << e.what() << std::endl;
        status = false;
    }

    input_data_file.close();

    return status;
}

// Create an Executor with the specified type and number of EOs
Executor* CreateExecutor(DeviceType dt, int num, const Configuration& c)
{
    if (num == 0) return nullptr;

    DeviceIds ids;
    for (int i = 0; i < num; i++)
        ids.insert(static_cast<DeviceId>(i));

    return new Executor(dt, ids, c);
}

// Accumulate EOs from an Executor into a vector of EOs
void CollectEOs(const Executor *e, vector<ExecutionObject *>& EOs)
{
    if (!e) return;

    for (unsigned int i = 0; i < e->GetNumExecutionObjects(); i++)
        EOs.push_back((*e)[i]);
}

