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

using namespace tinn;
using namespace tinn::imgutil;
using namespace cv;

#define NUM_VIDEO_FRAMES  100

bool RunConfiguration(const std::string& config_file, int num_devices,
                      DeviceType device_type, std::string& input_file);
bool RunAllConfigurations(int32_t num_devices, DeviceType device_type);

bool ReadFrame(ExecutionObject& eo, int frame_idx,
               const Configuration& configuration, int num_frames,
               std::string& image_file, VideoCapture &cap);

bool WriteFrameOutput(const ExecutionObject &eo);

static void ProcessArgs(int argc, char *argv[],
                        std::string& config,
                        int& num_devices,
                        DeviceType& device_type,
                        std::string& input_file);

static void DisplayHelp();

static double ms_diff(struct timespec &t0, struct timespec &t1)
{ return (t1.tv_sec - t0.tv_sec) * 1e3 + (t1.tv_nsec - t0.tv_nsec) / 1e6; }


int main(int argc, char *argv[])
{
    // Catch ctrl-c to ensure a clean exit
    signal(SIGABRT, exit);
    signal(SIGTERM, exit);

    // If there are no devices capable of offloading TIDL on the SoC, exit
    uint32_t num_dla = Executor::GetNumDevices(DeviceType::DLA);
    uint32_t num_dsp = Executor::GetNumDevices(DeviceType::DSP);
    if (num_dla == 0 && num_dsp == 0)
    {
        std::cout << "TI DL not supported on this SoC." << std::endl;
        return EXIT_SUCCESS;
    }

    // Process arguments
    std::string config      = "j11_v2";
    std::string input_file  = "../test/testvecs/input/objects/cat-pet-animal-domestic-104827.jpeg";
    int         num_devices = 1;
    DeviceType  device_type = DeviceType::DLA;
    ProcessArgs(argc, argv, config, num_devices, device_type, input_file);

    std::cout << "Input: " << input_file << std::endl;
    std::string config_file = "../test/testvecs/config/infer/tidl_config_"
                              + config + ".txt";
    bool status = RunConfiguration(config_file, num_devices, device_type,
                                   input_file);

    if (!status)
    {
        std::cout << "imagenet FAILED" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "imagenet PASSED" << std::endl;
    return EXIT_SUCCESS;
}

bool RunConfiguration(const std::string& config_file, int num_devices,
                      DeviceType device_type, std::string& input_file)
{
    DeviceIds ids;
    for (int i = 0; i < num_devices; i++)
        ids.insert(static_cast<DeviceId>(i));

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
    int num_frames = 1;
    VideoCapture cap;
    std::string image_file;
    if (input_file == "camera")
    {
        cap = VideoCapture(1);  // cap = VideoCapture("test.mp4");
        if (! cap.isOpened())
        {
            std::cerr << "Cannot open camera input." << std::endl;
            return false;
        }
        num_frames = NUM_VIDEO_FRAMES;
        namedWindow("ImageNet", WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    }
    else
    {
        image_file = input_file;
    }

    // Determine input frame size from configuration
    size_t frame_sz = configuration.inWidth * configuration.inHeight *
                      configuration.inNumChannels;

    try
    {
        // Create a executor with the approriate core type, number of cores
        // and configuration specified
        Executor executor(device_type, ids, configuration);

        // Query Executor for set of ExecutionObjects created
        const ExecutionObjects& execution_objects =
                                                executor.GetExecutionObjects();
        int num_eos = execution_objects.size();

        // Allocate input and output buffers for each execution object
        std::vector<void *> buffers;
        for (auto &eo : execution_objects)
        {
            ArgInfo in  = { ArgInfo(malloc(frame_sz), frame_sz)};
            ArgInfo out = { ArgInfo(malloc(frame_sz), frame_sz)};
            eo->SetInputOutputBuffer(in, out);

            buffers.push_back(in.ptr());
            buffers.push_back(out.ptr());
        }

        #define MAX_NUM_EOS  4
        struct timespec t0[MAX_NUM_EOS], t1;

        // Process frames with available execution objects in a pipelined manner
        // additional num_eos iterations to flush the pipeline (epilogue)
        for (int frame_idx = 0;
             frame_idx < num_frames + num_eos; frame_idx++)
        {
            ExecutionObject* eo = execution_objects[frame_idx % num_eos].get();

            // Wait for previous frame on the same eo to finish processing
            if (eo->ProcessFrameWait())
            {
                clock_gettime(CLOCK_MONOTONIC, &t1);
                double elapsed_host =
                                ms_diff(t0[eo->GetFrameIndex() % num_eos], t1);
                double elapsed_device = eo->GetProcessTimeInMilliSeconds();
                double overhead = 100 - (elapsed_device/elapsed_host*100);

                std::cout << "frame[" << eo->GetFrameIndex() << "]: "
                          << "Time on device: "
                          << std::setw(6) << std::setprecision(4)
                          << elapsed_device << "ms, "
                          << "host: "
                          << std::setw(6) << std::setprecision(4)
                          << elapsed_host << "ms ";
                std::cout << "API overhead: "
                          << std::setw(6) << std::setprecision(3)
                          << overhead << " %" << std::endl;

                WriteFrameOutput(*eo);
            }

            // Read a frame and start processing it with current eo
            if (ReadFrame(*eo, frame_idx, configuration, num_frames,
                          image_file, cap))
            {
                clock_gettime(CLOCK_MONOTONIC, &t0[frame_idx % num_eos]);
                eo->ProcessFrameStartAsync();
            }
        }

        for (auto b : buffers)
            free(b);

    }
    catch (tinn::Exception &e)
    {
        std::cerr << e.what() << std::endl;
        status = false;
    }

    return status;
}


bool ReadFrame(ExecutionObject &eo, int frame_idx,
               const Configuration& configuration, int num_frames,
               std::string& image_file, VideoCapture &cap)
{
    if (frame_idx >= num_frames)
        return false;
    eo.SetFrameIndex(frame_idx);

    char*  frame_buffer = eo.GetInputBufferPtr();
    assert (frame_buffer != nullptr);

    Mat image;
    if (! image_file.empty())
    {
        image = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
        if (image.empty())
        {
            std::cerr << "Unable to read from: " << image_file << std::endl;
            return false;
        }
    }
    else
    {
        Mat v_image;
        if (! cap.grab())  return false;
        if (! cap.retrieve(v_image)) return false;
        // Crop 640x480 camera input to center 256x256 input
        image = Mat(v_image, Rect(192, 112, 256, 256));
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
                  << ", prob = " << (float) sorted[i].first / 256 << std::endl;
    }

    return true;
}


void ProcessArgs(int argc, char *argv[], std::string& config,
                 int& num_devices, DeviceType& device_type,
                 std::string& input_file)
{
    const struct option long_options[] =
    {
        {"config",      required_argument, 0, 'c'},
        {"num_devices", required_argument, 0, 'n'},
        {"device_type", required_argument, 0, 't'},
        {"image_file",  required_argument, 0, 'i'},
        {"help",        no_argument,       0, 'h'},
        {"verbose",     no_argument,       0, 'v'},
        {0, 0, 0, 0}
    };

    int option_index = 0;

    while (true)
    {
        int c = getopt_long(argc, argv, "c:n:t:i:hv", long_options, &option_index);

        if (c == -1)
            break;

        switch (c)
        {
            case 'c': config = optarg;
                      break;

            case 'n': num_devices = atoi(optarg);
                      assert (num_devices > 0 && num_devices <= 4);
                      break;

            case 't': if (*optarg == 'e')
                          device_type = DeviceType::DLA;
                      else if (*optarg == 'd')
                          device_type = DeviceType::DSP;
                      else
                      {
                          std::cerr << "Invalid argument to -t, only e or d"
                                       " allowed" << std::endl;
                          exit(EXIT_FAILURE);
                      }
                      break;

            case 'i': input_file = optarg;
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
                 " -n <number of cores> Number of cores to use (1 - 4)\n"
                 " -t <d|e>             Type of core. d -> DSP, e -> DLA\n"
                 " -i <image>           Path to the image file\n"
                 " -i camera            Use camera as input\n"
                 " -v                   Verbose output during execution\n"
                 " -h                   Help\n";
}

