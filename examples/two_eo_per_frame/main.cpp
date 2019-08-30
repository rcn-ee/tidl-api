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
// This example illustrates using multiple EOs to process a single frame
// For details, refer http://downloads.ti.com/mctools/esd/docs/tidl-api/
//
#include <signal.h>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>

#include "executor.h"
#include "execution_object.h"
#include "execution_object_pipeline.h"
#include "configuration.h"
#include "utils.h"

using namespace tidl;
using std::string;
using std::unique_ptr;
using std::vector;

using EOP = tidl::ExecutionObjectPipeline;

bool Run(int num_eve,int num_dsp, const char* ref_output);

int main(int argc, char *argv[])
{
    // Catch ctrl-c to ensure a clean exit
    signal(SIGABRT, exit);
    signal(SIGTERM, exit);

    // This example requires both EVE and C66x
    uint32_t num_eve = Executor::GetNumDevices(DeviceType::EVE);
    uint32_t num_dsp = Executor::GetNumDevices(DeviceType::DSP);
    if (num_eve == 0 || num_dsp == 0)
    {
        std::cout << "TI DL not supported on this SoC." << std::endl;
        return EXIT_SUCCESS;
    }

    string ref_file ="../test/testvecs/reference/j11_v2_ref.bin";
    unique_ptr<const char> reference_output(ReadReferenceOutput(ref_file));

    bool status = Run(num_eve, num_dsp, reference_output.get());

    if (!status)
    {
        std::cout << "FAILED" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "PASSED" << std::endl;
    return EXIT_SUCCESS;
}

bool Run(int num_eve, int num_dsp, const char* ref_output)
{
    string config_file ="../test/testvecs/config/infer/tidl_config_j11_v2.txt";

    Configuration c;
    if (!c.ReadFromFile(config_file))
        return false;

    // Heap sizes for this network determined using Configuration::showHeapStats
    c.PARAM_HEAP_SIZE   = (3 << 20); // 3MB
    c.NETWORK_HEAP_SIZE = (34 << 20); // 34MB

    // Run this example for 16 input frames
    c.numFrames = 16;

    // Assign layers 12, 13 and 14 to the DSP layer group
    const int EVE_LG = 1;
    const int DSP_LG = 2;
    c.layerIndex2LayerGroupId = { {12, DSP_LG}, {13, DSP_LG}, {14, DSP_LG} };

    // Open input file for reading
    std::ifstream input(c.inData, std::ios::binary);

    bool status = true;
    try
    {
        // Create Executors - use all the DSP and EVE cores available
        // Specify layer group id for each Executor
        unique_ptr<Executor> eve(CreateExecutor(DeviceType::EVE,
                                                num_eve, c, EVE_LG));
        unique_ptr<Executor> dsp(CreateExecutor(DeviceType::DSP,
                                                num_dsp, c, DSP_LG));

        // Create pipelines. Each pipeline has 1 EVE and 1 DSP. If there are
        // more EVEs than DSPs, the DSPs are shared across multiple
        // pipelines. E.g.
        // 2 EVE, 2 DSP: EVE1 -> DSP1, EVE2 -> DSP2
        // 4 EVE, 2 DSP: EVE1 -> DSP1, EVE2 -> DSP2, EVE3 -> DSP1, EVE4 ->DSP2
        std::vector<EOP *> EOPs;
        uint32_t num_pipe = std::max(num_eve, num_dsp);
        for (uint32_t i = 0; i < num_pipe; i++)
              EOPs.push_back(new EOP( { (*eve)[i % num_eve],
                                        (*dsp)[i % num_dsp] } ));

        AllocateMemory(EOPs);

        // Process frames with EOs in a pipelined manner
        // additional num_eos iterations to flush the pipeline (epilogue)
        int num_eops = EOPs.size();
        for (int frame_idx = 0; frame_idx < c.numFrames + num_eops; frame_idx++)
        {
            EOP* eop = EOPs[frame_idx % num_eops];

            // Wait for previous frame on the same EOP to finish processing
            if (eop->ProcessFrameWait())
            {
                // The reference output is valid only for the first frame
                // processed on each EOP
                if (frame_idx < num_eops && !CheckFrame(eop, ref_output))
                    status = false;
            }

            // Read a frame and start processing it with current eo
            if (ReadFrame(eop, frame_idx, c, input))
                eop->ProcessFrameStartAsync();
        }

        FreeMemory(EOPs);

    }
    catch (tidl::Exception &e)
    {
        std::cerr << e.what() << std::endl;
        status = false;
    }

    input.close();

    return status;
}


