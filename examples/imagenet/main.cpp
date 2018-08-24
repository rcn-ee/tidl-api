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
#include <getopt.h>
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

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"


bool __TI_show_debug_ = false;

using namespace tidl;
using namespace tidl::imgutil;
using namespace cv;

#define NUM_VIDEO_FRAMES  300
#define NUM_DEFAULT_INPUTS  1
const char *default_inputs[NUM_DEFAULT_INPUTS] =
{
    "../test/testvecs/input/objects/cat-pet-animal-domestic-104827.jpeg"
};
bool is_camera_input = false;
bool is_video_input  = false;

bool RunConfiguration(const std::string& config_file,
                      uint32_t num_dsps, uint32_t num_eves,
                      std::string& input_file, int num_frames);
bool RunAllConfigurations(int32_t num_devices, DeviceType device_type);

void ReportTime(ExecutionObject& eo);
bool ReadFrame(ExecutionObject& eo, int frame_idx,
               const Configuration& configuration, int num_frames,
               const std::string& input_file, VideoCapture &cap);

bool WriteFrameOutput(const ExecutionObject &eo);

static
void ProcessArgs(int argc, char *argv[], std::string& config,
                 uint32_t& num_dsps, uint32_t& num_eves,
                 std::string& input_file, int &num_frames);
bool IsCameraOrVideoInput(const std::string& s);

static void DisplayHelp();


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
        std::cout << "TI DL not supported on this SoC." << std::endl;
        return EXIT_SUCCESS;
    }

    // Process arguments
    std::string config     = "j11_v2";
    std::string input_file;
    if (num_eves != 0) { num_eves = 1;  num_dsps = 0; }
    else               { num_eves = 0;  num_dsps = 1; }
    int         num_frames  = 1;
    ProcessArgs(argc, argv, config, num_dsps, num_eves, input_file, num_frames);

    assert(num_dsps != 0 || num_eves != 0);

    if (IsCameraOrVideoInput(input_file) && num_frames == 1)
        num_frames = NUM_VIDEO_FRAMES;
    if (input_file.empty())
        std::cout << "Input: " << default_inputs[0] << std::endl;
    else
        std::cout << "Input: " << input_file << std::endl;

    std::string config_file = "../test/testvecs/config/infer/tidl_config_"
                              + config + ".txt";
    bool status = RunConfiguration(config_file, num_dsps, num_eves,
                                   input_file, num_frames);
    if (!status)
    {
        std::cout << "imagenet FAILED" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "imagenet PASSED" << std::endl;
    return EXIT_SUCCESS;
}

bool IsCameraOrVideoInput(const std::string& s)
{
    is_camera_input = (s == "camera");
    is_video_input  = (s.size() > 4 && s.substr(s.size() - 4, 4) == ".mp4");
    return is_camera_input || is_video_input;
}

bool RunConfiguration(const std::string& config_file,
                      uint32_t num_dsps, uint32_t num_eves,
                      std::string& input_file, int num_frames)
{
    DeviceIds dsp_ids, eve_ids;
    for (uint32_t i = 0; i < num_dsps; i++)
        dsp_ids.insert(static_cast<DeviceId>(i));
    for (uint32_t i = 0; i < num_eves; i++)
        eve_ids.insert(static_cast<DeviceId>(i));

    // Read the TI DL configuration file
    Configuration configuration;
    bool status = configuration.ReadFromFile(config_file);
    if (!status)
    {
        std::cerr << "Error in configuration file: " << config_file
                  << std::endl;
        return false;
    }

    // setup input
    VideoCapture cap;
    if (is_camera_input || is_video_input)
    {
        if (is_camera_input)
            cap = VideoCapture(1);
        else
            cap = VideoCapture(input_file);
        if (! cap.isOpened())
        {
            std::cerr << "Cannot open video input: " << input_file << std::endl;
            return false;
        }
        namedWindow("ImageNet", WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    }

    try
    {
        // Create Executors with the approriate core type, number of cores
        // and configuration specified
        Executor *e_eve = (num_eves == 0) ? nullptr :
                         new Executor(DeviceType::EVE, eve_ids, configuration);
        Executor *e_dsp = (num_dsps == 0) ? nullptr :
                         new Executor(DeviceType::DSP, dsp_ids, configuration);

        // Get ExecutionObjects from Executors
        std::vector<ExecutionObject*> eos;
        for (uint32_t i = 0; i < num_eves; i++)  eos.push_back((*e_eve)[i]);
        for (uint32_t i = 0; i < num_dsps; i++)  eos.push_back((*e_dsp)[i]);
        int num_eos = eos.size();

        // Allocate input and output buffers for each ExecutionObject
        std::vector<void *> buffers;
        for (auto eo : eos)
        {
            size_t in_size  = eo->GetInputBufferSizeInBytes();
            size_t out_size = eo->GetOutputBufferSizeInBytes();
            void*  in_ptr   = malloc(in_size);
            void*  out_ptr  = malloc(out_size);
            assert(in_ptr != nullptr && out_ptr != nullptr);
            buffers.push_back(in_ptr);
            buffers.push_back(out_ptr);

            ArgInfo in  = { ArgInfo(in_ptr,  in_size)};
            ArgInfo out = { ArgInfo(out_ptr, out_size)};
            eo->SetInputOutputBuffer(in, out);
        }

        std::chrono::time_point<std::chrono::steady_clock> tloop0, tloop1;
        tloop0 = std::chrono::steady_clock::now();

        // Process frames with available execution objects in a pipelined manner
        // additional num_eos iterations to flush the pipeline (epilogue)
        for (int frame_idx = 0;
             frame_idx < num_frames + num_eos; frame_idx++)
        {
            ExecutionObject* eo = eos[frame_idx % num_eos];

            // Wait for previous frame on the same eo to finish processing
            if (eo->ProcessFrameWait())
            {
                ReportTime(*eo);
                WriteFrameOutput(*eo);
            }

            // Read a frame and start processing it with current eo
            if (ReadFrame(*eo, frame_idx, configuration, num_frames,
                          input_file, cap))
            {
                eo->ProcessFrameStartAsync();
            }
        }

        tloop1 = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsed = tloop1 - tloop0;
        std::cout << "Loop total time (including read/write/opencv/print/etc): "
                  << std::setw(6) << std::setprecision(4)
                  << (elapsed.count() * 1000) << "ms" << std::endl;

        for (auto b : buffers)
            free(b);
        delete e_eve;
        delete e_dsp;
    }
    catch (tidl::Exception &e)
    {
        std::cerr << e.what() << std::endl;
        status = false;
    }

    return status;
}

void ReportTime(ExecutionObject& eo)
{
    double elapsed_host   = eo.GetHostProcessTimeInMilliSeconds();
    double elapsed_device = eo.GetProcessTimeInMilliSeconds();
    double overhead = 100 - (elapsed_device/elapsed_host*100);

    std::cout << "frame[" << eo.GetFrameIndex() << "]: "
              << "Time on " << eo.GetDeviceName() << ": "
              << std::setw(6) << std::setprecision(4)
              << elapsed_device << "ms, "
              << "host: "
              << std::setw(6) << std::setprecision(4)
              << elapsed_host << "ms ";
    std::cout << "API overhead: "
              << std::setw(6) << std::setprecision(3)
              << overhead << " %" << std::endl;
}

bool ReadFrame(ExecutionObject &eo, int frame_idx,
               const Configuration& configuration, int num_frames,
               const std::string& input_file, VideoCapture &cap)
{
    if (frame_idx >= num_frames)
        return false;
    eo.SetFrameIndex(frame_idx);

    char*  frame_buffer = eo.GetInputBufferPtr();
    assert (frame_buffer != nullptr);

    Mat image;
    if (! is_camera_input && ! is_video_input)
    {
        if (input_file.empty())
            image = cv::imread(default_inputs[frame_idx % NUM_DEFAULT_INPUTS],
                               CV_LOAD_IMAGE_COLOR);
        else
            image = cv::imread(input_file, CV_LOAD_IMAGE_COLOR);
        if (image.empty())
        {
            std::cerr << "Unable to read input image" << std::endl;
            return false;
        }
    }
    else
    {
        Mat v_image;
        if (! cap.grab())  return false;
        if (! cap.retrieve(v_image)) return false;
        if (is_camera_input)
            // Crop 640x480 camera input to center 256x256 input
            image = Mat(v_image, Rect(192, 112, 256, 256));
        else
            image = v_image;
        cv::imshow("ImageNet", image);
        waitKey(2);
    }

    // TI DL image preprocessing, into frame_buffer
    return PreProcImage(image, frame_buffer, 1, 3,
                        configuration.inWidth, configuration.inHeight,
                        configuration.inWidth,
                        configuration.inWidth * configuration.inHeight,
                        1, configuration.preProcType);
}

// Display top 5 classified imagenet classes with probabilities
bool WriteFrameOutput(const ExecutionObject &eo)
{
    const int k = 5;
    unsigned char *out = (unsigned char *) eo.GetOutputBufferPtr();
    int out_size = eo.GetOutputBufferSizeInBytes();

    // sort and get k largest values and corresponding indices
    typedef std::pair<unsigned char, int> val_index;
    auto constexpr cmp = [](val_index &left, val_index &right)
                         { return left.first > right.first; };
    std::priority_queue<val_index, std::vector<val_index>, decltype(cmp)>
            queue(cmp);
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
    std::vector<val_index> sorted;
    while (! queue.empty())
    {
      sorted.push_back(queue.top());
      queue.pop();
    }

    for (int i = k - 1; i >= 0; i--)
    {
        std::cout << k-i << ": " << imagenet_classes[sorted[i].second]
                  << std::endl;
    }

    return true;
}


void ProcessArgs(int argc, char *argv[], std::string& config,
                 uint32_t& num_dsps, uint32_t& num_eves,
                 std::string& input_file, int &num_frames)
{
    const struct option long_options[] =
    {
        {"config",      required_argument, 0, 'c'},
        {"num_devices", required_argument, 0, 'n'},
        {"device_type", required_argument, 0, 't'},
        {"image_file",  required_argument, 0, 'i'},
        {"num_frames",  required_argument, 0, 'f'},
        {"help",        no_argument,       0, 'h'},
        {"verbose",     no_argument,       0, 'v'},
        {0, 0, 0, 0}
    };

    int option_index = 0;

    while (true)
    {
        int c = getopt_long(argc, argv, "c:d:e:i:f:hv", long_options, &option_index);

        if (c == -1)
            break;

        switch (c)
        {
            case 'c': config = optarg;
                      break;

            case 'd': num_dsps = atoi(optarg);
                      assert(num_dsps >= 0 && num_dsps <=
                                     Executor::GetNumDevices(DeviceType::DSP));
                      break;

            case 'e': num_eves = atoi(optarg);
                      assert(num_eves >= 0 && num_eves <=
                                     Executor::GetNumDevices(DeviceType::EVE));
                      break;

            case 'i': input_file = optarg;
                      break;

            case 'f': num_frames = atoi(optarg);
                      assert (num_frames > 0);
                      break;

            case 'v': __TI_show_debug_ = true;
                      break;

            case 'h': DisplayHelp();
                      exit(EXIT_SUCCESS);
                      break;

            case '?': // Error in getopt_long
                      exit(EXIT_FAILURE);
                      break;

            default:
                      std::cerr << "Unsupported option: " << c << std::endl;
                      break;
        }
    }
}

void DisplayHelp()
{
    std::cout << "Usage: imagenet\n"
                 "  Will run imagenet network to predict top 5 object"
                 " classes for the input.\n  Use -c to run a"
                 "  different imagenet network. Default is j11_v2.\n"
                 "Optional arguments:\n"
                 " -c <config>          Valid configs: j11_bn, j11_prelu, j11_v2\n"
                 " -d <number>          Number of dsp cores to use\n"
                 " -e <number>          Number of eve cores to use\n"
                 " -i <image>           Path to the image file\n"
                 " -i camera            Use camera as input\n"
                 " -i *.mp4             Use video file as input\n"
                 " -f <number>          Number of frames to process\n"
                 " -v                   Verbose output during execution\n"
                 " -h                   Help\n";
}

