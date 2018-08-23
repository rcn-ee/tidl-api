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
#include "configuration.h"
#include "imagenet_classes.h"
#include "imgutil.h"
#include "../common/video_utils.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

using namespace std;
using namespace tidl;
using namespace tidl::imgutil;
using namespace cv;

#define NUM_VIDEO_FRAMES  300
#define DEFAULT_CONFIG    "j11_v2"
#define NUM_DEFAULT_INPUTS  1
const char *default_inputs[NUM_DEFAULT_INPUTS] =
{
    "../test/testvecs/input/objects/cat-pet-animal-domestic-104827.jpeg"
};

Executor* CreateExecutor(DeviceType dt, int num, const Configuration& c);
bool RunConfiguration(cmdline_opts_t& opts);
bool ReadFrame(ExecutionObject& eo, int frame_idx, const Configuration& c,
               const cmdline_opts_t& opts, VideoCapture &cap);
bool WriteFrameOutput(const ExecutionObject &eo);
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
    if (opts.num_frames == 0)
        opts.num_frames = (opts.is_camera_input || opts.is_video_input) ?
                          NUM_VIDEO_FRAMES : 1;
    if (opts.input_file.empty())
        cout << "Input: " << default_inputs[0] << endl;
    else
        cout << "Input: " << opts.input_file << endl;

    // Run network
    bool status = RunConfiguration(opts);
    if (!status)
    {
        cout << "imagenet FAILED" << endl;
        return EXIT_FAILURE;
    }

    cout << "imagenet PASSED" << endl;
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

    // setup camera/video input/output
    VideoCapture cap;
    if (! SetVideoInputOutput(cap, opts, "ImageNet"))  return false;

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

        // Allocate input and output buffers for each ExecutionObject
        AllocateMemory(eos);

        chrono::time_point<chrono::steady_clock> tloop0, tloop1;
        tloop0 = chrono::steady_clock::now();

        // Process frames with available eos in a pipelined manner
        // additional num_eos iterations to flush the pipeline (epilogue)
        for (uint32_t frame_idx = 0;
             frame_idx < opts.num_frames + num_eos; frame_idx++)
        {
            ExecutionObject* eo = eos[frame_idx % num_eos];

            // Wait for previous frame on the same eo to finish processing
            if (eo->ProcessFrameWait())
            {
                ReportTime(eo);
                WriteFrameOutput(*eo);
            }

            // Read a frame and start processing it with current eo
            if (ReadFrame(*eo, frame_idx, c, opts, cap))
                eo->ProcessFrameStartAsync();
        }

        tloop1 = chrono::steady_clock::now();
        chrono::duration<float> elapsed = tloop1 - tloop0;
        cout << "Loop total time (including read/write/opencv/print/etc): "
                  << setw(6) << setprecision(4)
                  << (elapsed.count() * 1000) << "ms" << endl;

        FreeMemory(eos);
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
Executor* CreateExecutor(DeviceType dt, int num, const Configuration& c)
{
    if (num == 0) return nullptr;

    DeviceIds ids;
    for (uint32_t i = 0; i < num; i++)
        ids.insert(static_cast<DeviceId>(i));

    return new Executor(dt, ids, c);
}

bool ReadFrame(ExecutionObject &eo, int frame_idx, const Configuration& c,
               const cmdline_opts_t& opts, VideoCapture &cap)
{
    if (frame_idx >= opts.num_frames)
        return false;
    eo.SetFrameIndex(frame_idx);

    char*  frame_buffer = eo.GetInputBufferPtr();
    assert (frame_buffer != nullptr);

    Mat image;
    if (! opts.is_camera_input && ! opts.is_video_input)
    {
        if (opts.input_file.empty())
            image = cv::imread(default_inputs[frame_idx % NUM_DEFAULT_INPUTS],
                               CV_LOAD_IMAGE_COLOR);
        else
            image = cv::imread(opts.input_file, CV_LOAD_IMAGE_COLOR);
        if (image.empty())
        {
            cerr << "Unable to read input image" << endl;
            return false;
        }
    }
    else
    {
        Mat v_image;
        if (! cap.grab())  return false;
        if (! cap.retrieve(v_image)) return false;
        int orig_width  = v_image.cols;
        int orig_height = v_image.rows;
        // Crop camera/video input to center 256x256 input
        if (orig_width > 256 && orig_height > 256)
        {
            image = Mat(v_image, Rect((orig_width-256)/2, (orig_height-256)/2,
                                       256, 256));
        }
        else
            image = v_image;
        cv::imshow("ImageNet", image);
        waitKey(2);
    }

    // TI DL image preprocessing, into frame_buffer
    return PreProcImage(image, frame_buffer, 1, 3, c.inWidth, c.inHeight,
                        c.inWidth, c.inWidth * c.inHeight, 1, c.preProcType);
}

// Display top 5 classified imagenet classes with probabilities
bool WriteFrameOutput(const ExecutionObject &eo)
{
    const int k = 5;
    unsigned char *out = (unsigned char *) eo.GetOutputBufferPtr();
    int out_size = eo.GetOutputBufferSizeInBytes();

    // sort and get k largest values and corresponding indices
    typedef pair<unsigned char, int> val_index;
    auto constexpr cmp = [](val_index &left, val_index &right)
                         { return left.first > right.first; };
    priority_queue<val_index, vector<val_index>, decltype(cmp)> queue(cmp);
    // initialize priority queue with smallest value on top
    for (int i = 0; i < k; i++)
        queue.push(val_index(out[i], i));

    // for rest output, if larger than current min, pop min, push new val
    for (int i = k; i < out_size; i++)
    {
        if (out[i] > queue.top().first)
        {
          queue.pop();
          queue.push(val_index(out[i], i));
        }
    }

    // output top k values in reverse order: largest val first
    vector<val_index> sorted;
    while (! queue.empty())
    {
      sorted.push_back(queue.top());
      queue.pop();
    }

    for (int i = k - 1; i >= 0; i--)
        cout << k-i << ": " << imagenet_classes[sorted[i].second] << endl;

    return true;
}

void DisplayHelp()
{
    cout <<
    "Usage: imagenet\n"
    "  Will run imagenet network to predict top 5 object"
    " classes for the input.\n  Use -c to run a"
    "  different imagenet network. Default is j11_v2.\n"
    "Optional arguments:\n"
    " -c <config>          Valid configs: j11_bn, j11_prelu, j11_v2\n"
    " -d <number>          Number of dsp cores to use\n"
    " -e <number>          Number of eve cores to use\n"
    " -i <image>           Path to the image file as input\n"
    " -i camera<number>    Use camera as input\n"
    "                      video input port: /dev/video<number>\n"
    " -i <name>.{mp4,mov,avi}  Use video file as input\n"
    " -f <number>          Number of frames to process\n"
    " -v                   Verbose output during execution\n"
    " -h                   Help\n";
}

