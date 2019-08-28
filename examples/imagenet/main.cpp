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
#include "../common/object_classes.h"
#include "imgutil.h"
#include "../common/video_utils.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

using namespace std;
using namespace tidl;
using namespace cv;

#define NUM_VIDEO_FRAMES  300
#define DEFAULT_CONFIG    "j11_v2"
#define NUM_DEFAULT_INPUTS  1
#define DEFAULT_OBJECT_CLASSES_LIST_FILE "imagenet_objects.json"
#define DEFAULT_OUTPUT_PROB_THRESHOLD  5
const char *default_inputs[NUM_DEFAULT_INPUTS] =
{
    "../test/testvecs/input/objects/cat-pet-animal-domestic-104827.jpeg"
};
std::unique_ptr<ObjectClasses> object_classes;


Executor* CreateExecutor(DeviceType dt, uint32_t num, const Configuration& c);
bool RunConfiguration(cmdline_opts_t& opts);
bool ReadFrame(ExecutionObjectPipeline& eop,
               uint32_t frame_idx, const Configuration& c,
               const cmdline_opts_t& opts, VideoCapture &cap);
bool WriteFrameOutput(const ExecutionObjectPipeline &eop,
                      const cmdline_opts_t& opts);
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
    opts.object_classes_list_file = DEFAULT_OBJECT_CLASSES_LIST_FILE;
    opts.output_prob_threshold = DEFAULT_OUTPUT_PROB_THRESHOLD;
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

    // Get object classes list
    object_classes = std::unique_ptr<ObjectClasses>(
                             new ObjectClasses(opts.object_classes_list_file));
    if (object_classes->GetNumClasses() == 0)
    {
        cout << "No object classes defined for this config." << endl;
        return EXIT_FAILURE;
    }

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
                WriteFrameOutput(*eop, opts);
            }

            // Read a frame and start processing it with current eop
            if (ReadFrame(*eop, frame_idx, c, opts, cap))
                eop->ProcessFrameStartAsync();
        }

        tloop1 = chrono::steady_clock::now();
        chrono::duration<float> elapsed = tloop1 - tloop0;
        cout << "Loop total time (including read/write/opencv/print/etc): "
                  << setw(6) << setprecision(4)
                  << (elapsed.count() * 1000) << "ms" << endl;

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
               const cmdline_opts_t& opts, VideoCapture &cap)
{
    if (frame_idx >= opts.num_frames)
        return false;

    eop.SetFrameIndex(frame_idx);

    char*  frame_buffer = eop.GetInputBufferPtr();
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
    return imgutil::PreprocessImage(image, frame_buffer, c);
}

// Display top 5 classified imagenet classes with probabilities 5% or higher
bool WriteFrameOutput(const ExecutionObjectPipeline &eop,
                      const cmdline_opts_t& opts)
{
    const int k = 5;
    unsigned char *out = (unsigned char *) eop.GetOutputBufferPtr();
    int out_size = eop.GetOutputBufferSizeInBytes();
    // Tensorflow trained network outputs 1001 probabilities,
    // with 0-index being background, thus we need to subtract 1 when
    // reporting classified object from 1000 categories
    int background_offset = out_size == 1001 ? 1 : 0;

    // sort and get k largest values and corresponding indices
    typedef pair<unsigned char, int> val_index;
    auto cmp = [](val_index &left, val_index &right)
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

    unsigned int min_prob_255 = opts.output_prob_threshold * 255;
    for (int i = k - 1; i >= 0; i--)
    {
        if (sorted[i].first * 100 < min_prob_255)  break;
        int imagenet_index = sorted[i].second - background_offset;
        cout << k-i << ": [" << imagenet_index << "] "
             << object_classes->At(imagenet_index).label
             << ",   prob = " << setprecision(4)
             << ((sorted[i].first * 100) / 255.0f) << "%" << endl;
    }

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
    " -l <objects_list>    Path to the object classes list file\n"
    " -f <number>          Number of frames to process\n"
    " -p <number>          Output probablity threshold in percentage\n"
    "                      Default is 5 percent or higher.\n"
    " -v                   Verbose output during execution\n"
    " -h                   Help\n";
}

