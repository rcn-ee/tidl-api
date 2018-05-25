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
#include <cstdio>

#include "executor.h"
#include "execution_object.h"
#include "configuration.h"
#include "../segmentation/object_classes.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

#define NUM_VIDEO_FRAMES  100
#define DEFAULT_CONFIG    "jdetnet"
#define DEFAULT_INPUT     "../test/testvecs/input/preproc_0_768x320.y"

bool __TI_show_debug_ = false;
bool is_default_input = false;
bool is_preprocessed_input = false;
bool is_camera_input       = false;
int  orig_width;
int  orig_height;
object_class_table_t *object_class_table;

using namespace tidl;
using namespace cv;


bool RunConfiguration(const std::string& config_file, uint32_t num_devices,
                      DeviceType device_type, std::string& input_file);
bool ReadFrame(ExecutionObject& eo, int frame_idx,
               const Configuration& configuration, int num_frames,
               std::string& image_file, VideoCapture &cap);
bool WriteFrameOutput(const ExecutionObject &eo_in,
                      const ExecutionObject &eo_out,
                      const Configuration& configuration);

void ReportTime(int frame_index, std::string device_name, double elapsed_host,
                double elapsed_device);

static void ProcessArgs(int argc, char *argv[],
                        std::string& config,
                        uint32_t& num_devices,
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
    if (num_dla == 0 || num_dsp == 0)
    {
        std::cout << "ssd_multibox requires both DLA and DSP for execution."
                  << std::endl;
        return EXIT_SUCCESS;
    }

    // Process arguments
    std::string config      = DEFAULT_CONFIG;
    std::string input_file  = DEFAULT_INPUT;
    uint32_t num_devices    = 1;
    DeviceType  device_type = DeviceType::DLA;
    ProcessArgs(argc, argv, config, num_devices, device_type, input_file);

    // Use same number of DLAs and DSPs
    num_devices = std::min(num_devices, std::min(num_dla, num_dsp));
    if (num_devices == 0)
    {
        std::cout << "Partitioned execution requires at least 1 DLA and 1 DSP."
                  << std::endl;
        return EXIT_FAILURE;
    }
    if ((object_class_table = GetObjectClassTable(config)) == nullptr)
    {
        std::cout << "No object classes defined for this config." << std::endl;
        return EXIT_FAILURE;
    }

    if (input_file == DEFAULT_INPUT)  is_default_input = true;
    if (input_file == "camera")       is_camera_input = true;
    if (input_file.length() > 2 &&
        input_file.compare(input_file.length() - 2, 2, ".y") == 0)
        is_preprocessed_input = true;
    std::cout << "Input: " << input_file << std::endl;
    std::string config_file = "../test/testvecs/config/infer/tidl_config_"
                              + config + ".txt";
    bool status = RunConfiguration(config_file, num_devices, device_type,
                                   input_file);

    if (!status)
    {
        std::cout << "ssd_multibox FAILED" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "ssd_multibox PASSED" << std::endl;
    return EXIT_SUCCESS;
}

bool RunConfiguration(const std::string& config_file, uint32_t num_devices,
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
    int num_frames = is_default_input ? 3 : 1;
    VideoCapture cap;
    std::string image_file;
    if (is_camera_input)
    {
        cap = VideoCapture(1);  // cap = VideoCapture("test.mp4");
        if (! cap.isOpened())
        {
            std::cerr << "Cannot open camera input." << std::endl;
            return false;
        }
        num_frames = NUM_VIDEO_FRAMES;
        namedWindow("SSD_Multibox", WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    }
    else
    {
        image_file = input_file;
    }

    try
    {
        // Create a executor with the approriate core type, number of cores
        // and configuration specified
        // DLA will run layersGroupId 1 in the network, while
        // DSP will run layersGroupId 2 in the network
        Executor executor_dla(DeviceType::DLA, ids, configuration, 1);
        Executor executor_dsp(DeviceType::DSP, ids, configuration, 2);

        // Query Executor for set of ExecutionObjects created
        const ExecutionObjects& execution_objects_dla =
                                            executor_dla.GetExecutionObjects();
        const ExecutionObjects& execution_objects_dsp =
                                            executor_dsp.GetExecutionObjects();
        int num_eos = execution_objects_dla.size();

        // Allocate input and output buffers for each execution object
        // Note that "out" is both the output of eo_dla and the input of eo_dsp
        // This is how two layersGroupIds, 1 and 2, are tied together
        std::vector<void *> buffers;
        for (int i = 0; i < num_eos; i++)
        {
            ExecutionObject *eo_dla = execution_objects_dla[i].get();
            size_t in_size  = eo_dla->GetInputBufferSizeInBytes();
            size_t out_size = eo_dla->GetOutputBufferSizeInBytes();
            ArgInfo in  = { ArgInfo(malloc(in_size),  in_size)  };
            ArgInfo out = { ArgInfo(malloc(out_size), out_size) };
            eo_dla->SetInputOutputBuffer(in, out);

            ExecutionObject *eo_dsp = execution_objects_dsp[i].get();
            size_t out2_size = eo_dsp->GetOutputBufferSizeInBytes();
            ArgInfo out2 = { ArgInfo(malloc(out2_size), out2_size) };
            eo_dsp->SetInputOutputBuffer(out, out2);

            buffers.push_back(in.ptr());
            buffers.push_back(out.ptr());
            buffers.push_back(out2.ptr());
        }

        #define MAX_NUM_EOS  4
        struct timespec t0[MAX_NUM_EOS], t1, tloop0, tloop1;
        clock_gettime(CLOCK_MONOTONIC, &tloop0);

        // Process frames with available execution objects in a pipelined manner
        // additional num_eos iterations to flush the pipeline (epilogue)
        ExecutionObject *eo_dla, *eo_dsp, *eo_input;
        for (int frame_idx = 0;
             frame_idx < num_frames + num_eos; frame_idx++)
        {
            eo_dla = execution_objects_dla[frame_idx % num_eos].get();
            eo_dsp = execution_objects_dsp[frame_idx % num_eos].get();

            // Wait for previous frame on the same eo to finish processing
            if (eo_dsp->ProcessFrameWait())
            {
                int finished_idx = eo_dsp->GetFrameIndex();
                clock_gettime(CLOCK_MONOTONIC, &t1);
                ReportTime(finished_idx, "DSP",
                           ms_diff(t0[finished_idx % num_eos], t1),
                           eo_dsp->GetProcessTimeInMilliSeconds());

                eo_input = execution_objects_dla[finished_idx % num_eos].get();
                WriteFrameOutput(*eo_input, *eo_dsp, configuration);
            }

            // Read a frame and start processing it with current eo
            if (ReadFrame(*eo_dla, frame_idx, configuration, num_frames,
                          image_file, cap))
            {
                clock_gettime(CLOCK_MONOTONIC, &t0[frame_idx % num_eos]);
                eo_dla->ProcessFrameStartAsync();

                if (eo_dla->ProcessFrameWait())
                {
                    clock_gettime(CLOCK_MONOTONIC, &t1);
                    ReportTime(frame_idx, "DLA",
                               ms_diff(t0[frame_idx % num_eos], t1),
                               eo_dla->GetProcessTimeInMilliSeconds());

                    clock_gettime(CLOCK_MONOTONIC, &t0[frame_idx % num_eos]);
                    eo_dsp->ProcessFrameStartAsync();
                }
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &tloop1);
        std::cout << "Loop total time (including read/write/print/etc): "
                  << std::setw(6) << std::setprecision(4)
                  << ms_diff(tloop0, tloop1) << "ms" << std::endl;

        for (auto b : buffers)
            free(b);
    }
    catch (tidl::Exception &e)
    {
        std::cerr << e.what() << std::endl;
        status = false;
    }

    return status;
}

void ReportTime(int frame_index, std::string device_name, double elapsed_host,
                double elapsed_device)
{
    double overhead = 100 - (elapsed_device/elapsed_host*100);
    std::cout << "frame[" << frame_index << "]: "
              << "Time on " << device_name << ": "
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
               std::string& image_file, VideoCapture &cap)
{
    if (frame_idx >= num_frames)
        return false;
    eo.SetFrameIndex(frame_idx);

    char*  frame_buffer = eo.GetInputBufferPtr();
    assert (frame_buffer != nullptr);
    int channel_size = configuration.inWidth * configuration.inHeight;

    Mat image;
    if (! image_file.empty())
    {
        if (is_preprocessed_input)
        {
            std::ifstream ifs(image_file, std::ios::binary);
            ifs.seekg(frame_idx * channel_size * 3);
            ifs.read(frame_buffer, channel_size * 3);
            bool ifs_status = ifs.good();
            ifs.close();
            orig_width  = configuration.inWidth;
            orig_height = configuration.inHeight;
            return ifs_status;  // already PreProc-ed
        }
        else
        {
            image = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
            if (image.empty())
            {
                std::cerr << "Unable to read from: " << image_file << std::endl;
                return false;
            }
        }
    }
    else
    {
        // 640x480 camera input, process one in every 5 frames,
        // can adjust number of skipped frames to match real time processing
        if (! cap.grab())  return false;
        if (! cap.grab())  return false;
        if (! cap.grab())  return false;
        if (! cap.grab())  return false;
        if (! cap.grab())  return false;
        if (! cap.retrieve(image)) return false;
    }

    // scale to network input size
    Mat s_image, bgr_frames[3];
    orig_width  = image.cols;
    orig_height = image.rows;
    cv::resize(image, s_image,
               Size(configuration.inWidth, configuration.inHeight),
               0, 0, cv::INTER_AREA);
    cv::split(s_image, bgr_frames);
    memcpy(frame_buffer,                bgr_frames[0].ptr(), channel_size);
    memcpy(frame_buffer+1*channel_size, bgr_frames[1].ptr(), channel_size);
    memcpy(frame_buffer+2*channel_size, bgr_frames[2].ptr(), channel_size);
    return true;
}

// Create frame with boxes drawn around classified objects
bool WriteFrameOutput(const ExecutionObject &eo_in,
                      const ExecutionObject &eo_out,
                      const Configuration& configuration)
{
    // Asseembly original frame
    int width  = configuration.inWidth;
    int height = configuration.inHeight;
    int channel_size = width * height;
    Mat frame, r_frame, bgr[3];

    unsigned char *in = (unsigned char *) eo_in.GetInputBufferPtr();
    bgr[0] = Mat(height, width, CV_8UC(1), in);
    bgr[1] = Mat(height, width, CV_8UC(1), in + channel_size);
    bgr[2] = Mat(height, width, CV_8UC(1), in + channel_size*2);
    cv::merge(bgr, 3, frame);

    int frame_index = eo_in.GetFrameIndex();
    char outfile_name[64];
    if (! is_camera_input && is_preprocessed_input)
    {
        snprintf(outfile_name, 64, "frame_%d.png", frame_index);
        cv::imwrite(outfile_name, frame);
        printf("Saving frame %d to: %s\n", frame_index, outfile_name);
    }

    // Draw boxes around classified objects
    float *out = (float *) eo_out.GetOutputBufferPtr();
    int num_floats = eo_out.GetOutputBufferSizeInBytes() / sizeof(float);
    for (int i = 0; i < num_floats / 7; i++)
    {
        int index = (int)    out[i * 7 + 0];
        if (index < 0)  break;

        int   label = (int)  out[i * 7 + 1];
        float score =        out[i * 7 + 2];
        int   xmin  = (int) (out[i * 7 + 3] * width);
        int   ymin  = (int) (out[i * 7 + 4] * height);
        int   xmax  = (int) (out[i * 7 + 5] * width);
        int   ymax  = (int) (out[i * 7 + 6] * height);

        object_class_t *object_class = GetObjectClass(object_class_table,
                                                      label);
        if (object_class == nullptr)  continue;

#if 0
        printf("(%d, %d) -> (%d, %d): %s, score=%f\n",
               xmin, ymin, xmax, ymax, object_class->label, score);
#endif

        cv::rectangle(frame, Point(xmin, ymin), Point(xmax, ymax),
                      Scalar(object_class->color.blue,
                             object_class->color.green,
                             object_class->color.red), 2);
    }

    // output
    cv::resize(frame, r_frame, Size(orig_width, orig_height));
    if (is_camera_input)
    {
        cv::imshow("SSD_Multibox", r_frame);
        waitKey(1);
    }
    else
    {
        snprintf(outfile_name, 64, "multibox_%d.png", frame_index);
        cv::imwrite(outfile_name, r_frame);
        printf("Saving frame %d with SSD multiboxes to: %s\n",
               frame_index, outfile_name);
    }

    return true;
}


void ProcessArgs(int argc, char *argv[], std::string& config,
                 uint32_t& num_devices, DeviceType& device_type,
                 std::string& input_file)
{
    const struct option long_options[] =
    {
        {"config",      required_argument, 0, 'c'},
        {"num_devices", required_argument, 0, 'n'},
        {"image_file",  required_argument, 0, 'i'},
        {"help",        no_argument,       0, 'h'},
        {"verbose",     no_argument,       0, 'v'},
        {0, 0, 0, 0}
    };

    int option_index = 0;

    while (true)
    {
        int c = getopt_long(argc, argv, "c:n:i:hv", long_options, &option_index);

        if (c == -1)
            break;

        switch (c)
        {
            case 'c': config = optarg;
                      break;

            case 'n': num_devices = atoi(optarg);
                      assert (num_devices > 0 && num_devices <= 4);
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
    std::cout << "Usage: ssd_multibox\n"
                 "  Will run partitioned ssd_multibox network to perform "
                 "multi-objects detection\n"
                 "  and classification.  First part of network "
                 "(layersGroupId 1) runs on DLA,\n"
                 "  second part (layersGroupId 2) runs on DSP.\n"
                 "  Use -c to run a different segmentation network. "
                 "Default is jdetnet.\n"
                 "Optional arguments:\n"
                 " -c <config>          Valid configs: jdetnet \n"
                 " -n <number of cores> Number of cores to use (1 - 4)\n"
                 " -i <image>           Path to the image file\n"
                 "                      Default is 1 frame in testvecs\n"
                 " -i camera            Use camera as input\n"
                 " -v                   Verbose output during execution\n"
                 " -h                   Help\n";
}

