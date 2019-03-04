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
#include <chrono>

#include "executor.h"
#include "execution_object.h"
#include "execution_object_pipeline.h"
#include "configuration.h"
#include "imgutil.h"
#include "../common/video_utils.h"

using namespace std;
using namespace tidl;

#define DEFAULT_CONFIG    "mnist_lenet"
#define DEFAULT_INPUT_IMAGES "../test/testvecs/input/digits10_images_28x28.y"
#define DEFAULT_INPUT_LABELS "../test/testvecs/input/digits10_labels_10x1.y"

uint32_t images_file_offset = 0;
uint32_t labels_file_offset = 0;
uint32_t num_frames_file    = 0;
uint32_t num_wrong_results  = 0;


Executor* CreateExecutor(DeviceType dt, uint32_t num, const Configuration& c);
bool RunConfiguration(cmdline_opts_t& opts);
bool ReadFrame(ExecutionObjectPipeline& eop,
               uint32_t frame_idx, const Configuration& c,
               const cmdline_opts_t& opts, ifstream &ifs);
bool WriteFrameOutput(const ExecutionObjectPipeline &eop, ifstream &ifs_labels);
void DisplayHelp();


int main(int argc, char *argv[])
{
    // Catch ctrl-c to ensure a clean exit
    signal(SIGABRT, exit);
    signal(SIGTERM, exit);

    // If there are no devices capable of offloading TIDL on the SoC, exit
    uint32_t num_eves = Executor::GetNumDevices(DeviceType::EVE);
    uint32_t num_dsps = Executor::GetNumDevices(DeviceType::DSP);
    if (num_eves == 0 && num_dsps == 0)
    {
        cout << "TI DL not supported on this SoC." << endl;
        return EXIT_SUCCESS;
    }

    // Process arguments
    cmdline_opts_t opts;
    opts.config = DEFAULT_CONFIG;
    if (num_eves != 0) { opts.num_eves = 1;  opts.num_dsps = 0; }
    else               { opts.num_eves = 0;  opts.num_dsps = 1; }
    if (! ProcessArgs(argc, argv, opts))
    {
        DisplayHelp();
        exit(EXIT_SUCCESS);
    }
    assert(opts.num_dsps != 0 || opts.num_eves != 0);

    if (opts.input_file.empty())
    {
        opts.input_file               = DEFAULT_INPUT_IMAGES;
        opts.object_classes_list_file = DEFAULT_INPUT_LABELS;
    }

    // if inputs are MNIST data set: skip MNIST header
    string& s_images = opts.input_file;
    if (s_images.size() > 10 &&
        s_images.compare(s_images.size() - 10, 10, "idx3-ubyte") == 0)
        images_file_offset = 16;
    string& s_labels = opts.object_classes_list_file;
    if (s_labels.size() > 10 &&
        s_labels.compare(s_labels.size() - 10, 10, "idx1-ubyte") == 0)
        labels_file_offset = 8;

    cout << "Input images: " << opts.input_file << endl;
    if (! opts.object_classes_list_file.empty())
        cout << "Input labels: " << opts.object_classes_list_file << endl;

    // Run network
    bool status = RunConfiguration(opts);
    if (!status)
    {
        cout << "mnist FAILED" << endl;
        return EXIT_FAILURE;
    }

    cout << "mnist PASSED" << endl;
    return EXIT_SUCCESS;
}

bool RunConfiguration(cmdline_opts_t& opts)
{
    // Read the TI DL configuration file
    Configuration c;
    string config_file = "../test/testvecs/config/infer/tidl_config_"
                         + opts.config + ".txt";
    bool status = c.ReadFromFile(config_file);
    if (!status)
    {
        cerr << "Error in configuration file: " << config_file << endl;
        return false;
    }
    c.enableApiTrace = opts.verbose;

    // setup images/labels input/output
    ifstream ifs, ifs_labels;
    ifs.open(opts.input_file, ios::binary | ios::ate);
    if (! ifs.good())
    {
        cerr << "Cannot open " << opts.input_file << endl;
        return false;
    }
    num_frames_file = (((int) ifs.tellg()) - images_file_offset) /
                      (c.inWidth * c.inHeight);
    if (opts.num_frames == 0)
        opts.num_frames = num_frames_file;
    if (! opts.object_classes_list_file.empty())
    {
        ifs_labels.open(opts.object_classes_list_file, ios::binary);
        if (! ifs_labels.good())
        {
            cerr << "Cannot open " << opts.object_classes_list_file << endl;
            return false;
        }
    }

    try
    {
        // Create Executors with the approriate core type, number of cores
        // and configuration specified
        Executor* e_eve = CreateExecutor(DeviceType::EVE, opts.num_eves, c);
        Executor* e_dsp = CreateExecutor(DeviceType::DSP, opts.num_dsps, c);

        // Get ExecutionObjects from Executors
        vector<ExecutionObject*> eos;
        for (uint32_t i = 0; i < opts.num_eves; i++) eos.push_back((*e_eve)[i]);
        for (uint32_t i = 0; i < opts.num_dsps; i++) eos.push_back((*e_dsp)[i]);
        uint32_t num_eos = eos.size();

        // Use duplicate EOPs to do double buffering on frame input/output
        //    because each EOP has its own set of input/output buffers,
        //    so that host ReadFrame() can be overlapped with device processing
        // Use one EO as an example, with different buffer_factor,
        //    we have different execution behavior:
        // If buffer_factor is set to 1 -> single buffering
        //    we create one EOP: eop0 (eo0)
        //    pipeline execution of multiple frames over time is as follows:
        //    --------------------- time ------------------->
        //    eop0: [RF][eo0.....][WF]
        //    eop0:                   [RF][eo0.....][WF]
        //    eop0:                                     [RF][eo0.....][WF]
        // If buffer_factor is set to 2 -> double buffering
        //    we create two EOPs: eop0 (eo0), eop1(eo0)
        //    pipeline execution of multiple frames over time is as follows:
        //    --------------------- time ------------------->
        //    eop0: [RF][eo0.....][WF]
        //    eop1:     [RF]      [eo0.....][WF]
        //    eop0:                   [RF]  [eo0.....][WF]
        //    eop1:                             [RF]  [eo0.....][WF]
        vector<ExecutionObjectPipeline *> eops;
        uint32_t buffer_factor = 2;  // set to 1 for single buffering
        for (uint32_t j = 0; j < buffer_factor; j++)
            for (uint32_t i = 0; i < num_eos; i++)
                eops.push_back(new ExecutionObjectPipeline({eos[i]}));
        uint32_t num_eops = eops.size();

        // Allocate input and output buffers for each EOP
        AllocateMemory(eops);

        float device_time = 0.0f;
        chrono::time_point<chrono::steady_clock> tloop0, tloop1;
        tloop0 = chrono::steady_clock::now();

        // Process frames with available eops in a pipelined manner
        // additional num_eos iterations to flush the pipeline (epilogue)
        for (uint32_t frame_idx = 0;
             frame_idx < opts.num_frames + num_eops; frame_idx++)
        {
            ExecutionObjectPipeline* eop = eops[frame_idx % num_eops];

            // Wait for previous frame on the same eop to finish processing
            if (eop->ProcessFrameWait())
            {
                device_time +=
                      eos[frame_idx % num_eos]->GetProcessTimeInMilliSeconds();
                WriteFrameOutput(*eop, ifs_labels);
            }

            // Read a frame and start processing it with current eop
            if (ReadFrame(*eop, frame_idx, c, opts, ifs))
                eop->ProcessFrameStartAsync();
        }

        tloop1 = chrono::steady_clock::now();
        chrono::duration<float> elapsed = tloop1 - tloop0;
        cout << "Device total time: " << setw(6) << setprecision(4)
             << device_time << "ms" << endl;
        cout << "Loop total time (including read/write/print/etc): "
             << setw(6) << setprecision(4)
             << (elapsed.count() * 1000) << "ms" << endl;
        if (opts.num_frames > 0 && ifs_labels.is_open())
        {
            cout << "Accuracy: " << setw(6) << setprecision(4)
                 << (opts.num_frames-num_wrong_results)*100.f / opts.num_frames
                 << "%" << endl;
            if (opts.input_file == DEFAULT_INPUT_IMAGES && num_wrong_results >0)
                status = false;
        }

        FreeMemory(eops);
        for (auto eop : eops)  delete eop;
        delete e_eve;
        delete e_dsp;
    }
    catch (tidl::Exception &e)
    {
        cerr << e.what() << endl;
        status = false;
    }

    return status;
}

// Create an Executor with the specified type and number of EOs
Executor* CreateExecutor(DeviceType dt, uint32_t num, const Configuration& c)
{
    if (num == 0) return nullptr;

    DeviceIds ids;
    for (uint32_t i = 0; i < num; i++)
        ids.insert(static_cast<DeviceId>(i));

    return new Executor(dt, ids, c);
}

bool ReadFrame(ExecutionObjectPipeline &eop,
               uint32_t frame_idx, const Configuration& c,
               const cmdline_opts_t& opts, ifstream &ifs)
{
    if (frame_idx >= opts.num_frames)
        return false;

    eop.SetFrameIndex(frame_idx);

    char*  frame_buffer = eop.GetInputBufferPtr();
    assert (frame_buffer != nullptr);
    int channel_size = c.inWidth * c.inHeight;

    // already PreProc-ed white-on-black 28x28 frames
    ifs.seekg(images_file_offset + (frame_idx%num_frames_file) * channel_size);
    ifs.read(frame_buffer, channel_size);
    return ifs.good();
}

// Display top 5 classified imagenet classes with probabilities
bool WriteFrameOutput(const ExecutionObjectPipeline &eop, ifstream &ifs_labels)
{
    unsigned char *out = (unsigned char *) eop.GetOutputBufferPtr();
    int out_size = eop.GetOutputBufferSizeInBytes();

    unsigned char maxval = 0;
    int           maxloc = -1;
    for (int i = 0; i < out_size; i++)
    {
        // cout << (int) out[i] << " ";  // 10 probability outputs
        if (out[i] > maxval)
        {
            maxval = out[i];
            maxloc = i;
        }
    }
    cout << maxloc << endl;

    // check inference result against pre-determined label
    if (ifs_labels.is_open())
    {
        int frame_index = eop.GetFrameIndex();
        char label = -1;
        ifs_labels.seekg(labels_file_offset + (frame_index % num_frames_file));
        ifs_labels.read(&label, 1);
        if (maxloc != (int) label)
            num_wrong_results += 1;
    }

    return true;
}

void DisplayHelp()
{
    cout <<
    "Usage: mnist\n"
    "  Will run MNIST LeNet to predict the digit in a 28x28"
    " white-on-black image.\n  Use -c to run a"
    "  different MNIST network. Default is mnist_lenet.\n"
    "Optional arguments:\n"
    " -c <config>          Valid configs: mnist_lenet\n"
    " -d <number>          Number of dsp cores to use\n"
    " -e <number>          Number of eve cores to use\n"
    " -i <images>          Path to the MNIST white-on-black images file\n"
    " -l <labels>          Path to the MNIST labels file\n"
    " -f <number>          Number of frames to process\n"
    " -v                   Verbose output during execution\n"
    " -h                   Help\n";
}

