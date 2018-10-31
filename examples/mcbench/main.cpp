/******************************************************************************
 * Copyright (c) 2018, Texas Instruments Incorporated - http://www.ti.com/
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
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cassert>
#include <string>
#include <functional>
#include <algorithm>
#include <time.h>
#include <unistd.h>

#include <queue>
#include <vector>
#include <cstdio>
#include <chrono>

#include "executor.h"
#include "execution_object.h"
#include "execution_object_pipeline.h"
#include "configuration.h"
#include "../segmentation/object_classes.h"
#include "../common/utils.h"
#include "../common/video_utils.h"

using namespace std;
using namespace tidl;
using namespace cv;


#define NUM_VIDEO_FRAMES  100
#define DEFAULT_CONFIG    "../test/testvecs/config/infer/tidl_config_j11_v2.txt"

bool RunConfiguration(const cmdline_opts_t& opts);
Executor* CreateExecutor(DeviceType dt, uint32_t num, const Configuration& c,
                         int layers_group_id);
bool CreateExecutionObjectPipelines(uint32_t num_eves, uint32_t num_dsps,
                                    Configuration& configuration,
                                    uint32_t num_layers_groups,
                                    Executor*& e_eve, Executor*& e_dsp,
                                  std::vector<ExecutionObjectPipeline*>& eops);

void AllocateMemory(const std::vector<ExecutionObjectPipeline*>& eops);

bool ReadFrame(ExecutionObjectPipeline& eop, uint32_t frame_idx,
               const Configuration& c, const cmdline_opts_t& opts, char *input_frames_buffer);
static void DisplayHelp();


int main(int argc, char *argv[])
{
    // Catch ctrl-c to ensure a clean exit
    signal(SIGABRT, exit);
    signal(SIGTERM, exit);

    // If there are no devices capable of offloading TIDL on the SoC, exit
    uint32_t num_eves = Executor::GetNumDevices(DeviceType::EVE);
    uint32_t num_dsps = Executor::GetNumDevices(DeviceType::DSP);
    if ((num_eves == 0) || (num_dsps == 0))
    {
        cout << "mcbench requires EVE and/or DSP for execution." << endl;
        return EXIT_SUCCESS;
    }

    cout << "CMDLINE: ";
    for(int i = 0; i < argc; ++i) cout << argv[i] << " ";
    cout << endl;

    // Process arguments
    cmdline_opts_t opts;
    opts.config = DEFAULT_CONFIG;
    opts.num_eves = 0;
    opts.num_dsps = 2;
    if (! ProcessArgs(argc, argv, opts))
    {
        DisplayHelp();
        exit(EXIT_SUCCESS);
    }
    assert((opts.num_dsps + opts.num_eves) != 0);

    if (opts.num_frames == 0)
        opts.num_frames = NUM_VIDEO_FRAMES;

    // Run network
    bool status = RunConfiguration(opts);

    if (!status)
    {
        cout << "mcbench FAILED" << endl;
        return EXIT_FAILURE;
    }

    cout << "mcbench PASSED" << endl;
    return EXIT_SUCCESS;
}

bool RunConfiguration(const cmdline_opts_t& opts)
{
    // Read the TI DL configuration file
    Configuration c;
    if (!c.ReadFromFile(opts.config))
        return false;

    c.enableApiTrace = opts.verbose;
    if(opts.num_layers_groups == 1)
       c.runFullNet = true; //Force all layers to be in the same group

    std::string inputFile;
    if (opts.input_file.empty())
        inputFile   = c.inData;
    else
        inputFile = opts.input_file;

    int frame_size = c.inNumChannels * c.inWidth * c.inHeight;

    c.numFrames = GetBinaryFileSize (inputFile) / frame_size;

    cout << "Input: " << inputFile << " frames:" << c.numFrames << endl;

    // Read input file into memory buffer
    char *input_frame_buffer = new char[c.numFrames * frame_size]();
    ifstream ifs(inputFile, ios::binary);
    ifs.read(input_frame_buffer, c.numFrames * frame_size);
    if(!ifs.good()) {
       std::cout << "Invalid File input:" << inputFile << std::endl;
       return false;
    }

    bool status = true;
    try
    {
        Executor *e_eve = NULL;
        Executor *e_dsp = NULL;
        std::vector<ExecutionObjectPipeline *> eops;
        if (! CreateExecutionObjectPipelines(opts.num_eves, opts.num_dsps, c,
                                             opts.num_layers_groups,
                                             e_eve, e_dsp, eops))
            return false;

        // Allocate input/output memory for each EOP
        AllocateMemory(eops);

        chrono::time_point<chrono::steady_clock> tloop0, tloop1;
        tloop0 = chrono::steady_clock::now();

        // Process frames with available eops in a pipelined manner
        // additional num_eops iterations to flush pipeline (epilogue)
        uint32_t num_eops = eops.size();
        for (uint32_t frame_idx = 0;
             frame_idx < opts.num_frames + num_eops; frame_idx++)
        {
            ExecutionObjectPipeline* eop = eops[frame_idx % num_eops];

            // Wait for previous frame on the same eop to finish processing
            if (eop->ProcessFrameWait())
                ;

            // Read a frame and start processing it with current eo
            if (ReadFrame(*eop, frame_idx, c, opts, input_frame_buffer))
                eop->ProcessFrameStartAsync();
        }

        tloop1 = chrono::steady_clock::now();
        chrono::duration<float> elapsed = tloop1 - tloop0;
        cout << "Loop total time: "
                  << setw(6) << setprecision(4)
                  << (elapsed.count() * 1000) << "ms" << endl;
        cout << "FPS:" << opts.num_frames / elapsed.count() << endl;

        FreeMemory(eops);

        for (auto eop : eops)
            delete eop;

        delete e_eve;
        delete e_dsp;
    }
    catch (tidl::Exception &e)
    {
        cerr << e.what() << endl;
        status = false;
    }

    delete [] input_frame_buffer;
    return status;
}

// Create an Executor with the specified type and number of EOs
Executor* CreateExecutor(DeviceType dt, uint32_t num, const Configuration& c,
                         int layers_group_id)
{
    if (num == 0) return nullptr;

    DeviceIds ids;
    for (uint32_t i = 0; i < num; i++)
        ids.insert(static_cast<DeviceId>(i));

    return new Executor(dt, ids, c, layers_group_id);
}

bool CreateExecutionObjectPipelines(uint32_t num_eves, uint32_t num_dsps,
                                    Configuration& configuration,
                                    uint32_t num_layers_groups,
                                    Executor*& e_eve, Executor*& e_dsp,
                                    std::vector<ExecutionObjectPipeline*>& eops)
{
    DeviceIds ids_eve, ids_dsp;
    for (uint32_t i = 0; i < num_eves; i++)
        ids_eve.insert(static_cast<DeviceId>(i));
    for (uint32_t i = 0; i < num_dsps; i++)
        ids_dsp.insert(static_cast<DeviceId>(i));

    // Construct ExecutionObjectPipeline that utilizes multiple
    // ExecutionObjects to process a single frame, each ExecutionObject
    // processes one layerGroup of the network
    //
    // Pipeline depth can enable more optimized pipeline execution:
    // Given one EVE and one DSP as an example, with different
    //     buffer_factor, we have different execution behavior:
    // If buffer_factor is set to 1,
    //    we create one EOP: eop0 (eve0, dsp0)
    //    pipeline execution of multiple frames over time is as follows:
    //    --------------------- time ------------------->
    //    eop0: [eve0...][dsp0]
    //    eop0:                [eve0...][dsp0]
    //    eop0:                               [eve0...][dsp0]
    //    eop0:                                              [eve0...][dsp0]
    // If buffer_factor is set to 2,
    //    we create two EOPs: eop0 (eve0, dsp0), eop1(eve0, dsp0)
    //    pipeline execution of multiple frames over time is as follows:
    //    --------------------- time ------------------->
    //    eop0: [eve0...][dsp0]
    //    eop1:          [eve0...][dsp0]
    //    eop0:                   [eve0...][dsp0]
    //    eop1:                            [eve0...][dsp0]
    // Additional benefit of setting buffer_factor to 2 is that
    //    it can also overlap host ReadFrame() with device processing:
    //    --------------------- time ------------------->
    //    eop0: [RF][eve0...][dsp0]
    //    eop1:     [RF]     [eve0...][dsp0]
    //    eop0:                    [RF][eve0...][dsp0]
    //    eop1:                             [RF][eve0...][dsp0]
    const uint32_t buffer_factor = 2;

    switch(num_layers_groups)
    {
    case 1: // Single layers group
        e_eve = num_eves == 0 ? nullptr :
                new Executor(DeviceType::EVE, ids_eve, configuration);
        e_dsp = num_dsps == 0 ? nullptr :
                new Executor(DeviceType::DSP, ids_dsp, configuration);

        // Construct ExecutionObjectPipeline with single Execution Object to
        // process each frame. This is parallel processing of frames with
        // as many DSP and EVE cores that we have on hand.
        // If buffer_factor == 2, duplicating EOPs for double buffering
        // and overlapping host pre/post-processing with device processing
        for (uint32_t j = 0; j < buffer_factor; j++)
        {
            for (uint32_t i = 0; i < num_eves; i++)
                eops.push_back(new ExecutionObjectPipeline({(*e_eve)[i]}));
            for (uint32_t i = 0; i < num_dsps; i++)
                eops.push_back(new ExecutionObjectPipeline({(*e_dsp)[i]}));
        }
        break;

    case 2: // Two layers group
        // Create Executors with the approriate core type, number of cores
        // and configuration specified
        // EVE will run layersGroupId 1 in the network, while
        // DSP will run layersGroupId 2 in the network
        e_eve = num_eves == 0 ? nullptr :
                new Executor(DeviceType::EVE, ids_eve, configuration, 1);
        e_dsp = num_dsps == 0 ? nullptr :
                new Executor(DeviceType::DSP, ids_dsp, configuration, 2);

        // Construct ExecutionObjectPipeline that utilizes multiple
        // ExecutionObjects to process a single frame, each ExecutionObject
        // processes one layerGroup of the network
        // If buffer_factor == 2, duplicating EOPs for pipelining at
        // EO level rather than at EOP level, in addition to double buffering
        // and overlapping host pre/post-processing with device processing
        for (uint32_t j = 0; j < buffer_factor; j++)
            for (uint32_t i = 0; i < std::max(num_eves, num_dsps); i++)
                eops.push_back(new ExecutionObjectPipeline(
                                {(*e_eve)[i%num_eves], (*e_dsp)[i%num_dsps]}));
        break;

    default:
        std::cout << "Layers groups must be either 1 or 2!" << std::endl;
        return false;
        break;
    }

    return true;
}

static void subtractMeanValue(unsigned char *frame_buffer, int channel_size,
                              int32_t mean_value)
{
    int32_t one_pixel;

    for (int i = 0; i < channel_size; i ++)
    {
        one_pixel  = (int32_t)frame_buffer[i];
        one_pixel -= mean_value;
        if(one_pixel > 127)  one_pixel = 127;
        if(one_pixel < -128) one_pixel = -128;
        frame_buffer[i] = (unsigned char)one_pixel;
    }
}

bool ReadFrame(ExecutionObjectPipeline& eop, uint32_t frame_idx,
               const Configuration& c, const cmdline_opts_t& opts,
               char *input_frames_buffer)
{
    if (frame_idx >= opts.num_frames)
        return false;

    eop.SetFrameIndex(frame_idx);

    unsigned char* frame_buffer = (unsigned char *)eop.GetInputBufferPtr();
    assert (frame_buffer != nullptr);
    //Current implementation of this function assumes 3 channels on input
    assert (c.inNumChannels == 3);

    int channel_size = c.inWidth * c.inHeight;
    char *bgr_frames_input = input_frames_buffer + (frame_idx % c.numFrames) *
                             channel_size * c.inNumChannels;

    memcpy(frame_buffer,                bgr_frames_input + 0, channel_size);
    if(c.preProcType == 1)
       subtractMeanValue(frame_buffer, channel_size, 104);
    else if(c.preProcType == 2)
       subtractMeanValue(frame_buffer, channel_size, 128);
    frame_buffer += channel_size;

    memcpy(frame_buffer, bgr_frames_input + 1 * channel_size, channel_size);
    if(c.preProcType == 1)
       subtractMeanValue(frame_buffer, channel_size, 117);
    else if(c.preProcType == 2)
       subtractMeanValue(frame_buffer, channel_size, 128);
    frame_buffer += channel_size;

    memcpy(frame_buffer, bgr_frames_input + 2 * channel_size, channel_size);
    if(c.preProcType == 1)
       subtractMeanValue(frame_buffer, channel_size, 123);
    else if(c.preProcType == 2)
       subtractMeanValue(frame_buffer, channel_size, 128);

    return true;
}

void DisplayHelp()
{
    std::cout <<
    "Usage: mcbench\n"
    "  Runs partitioned network to perform multi-object detection\n"
    "  and classification. First part of network (layersGroupId 1) runs on\n"
    "  EVE, second part (layersGroupId 2) runs on DSP.\n"
    "  Use -c to run a different segmentation network.  Default is jdetnet.\n"
    "Optional arguments:\n"
    " -c <config>          Valid configs: ../test/testvecs/config/infer/... \n"
    " -d <number>          Number of DSP cores to use\n"
    " -e <number>          Number of EVE cores to use\n"
    " -g <1|2>             Number of layer groups\n"
    " -f <number>          Number of frames to process\n"
    " -i <image>           Path to the input image file\n"
    " -v                   Verbose output during execution\n"
    " -h                   Help\n";
}

