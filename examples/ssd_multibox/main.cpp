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
#include <string>
#include <chrono>

#include "executor.h"
#include "execution_object.h"
#include "execution_object_pipeline.h"
#include "configuration.h"
#include "../common/object_classes.h"
#include "../common/utils.h"
#include "../common/video_utils.h"

using namespace std;
using namespace tidl;
using namespace cv;


#define NUM_VIDEO_FRAMES  100
#define DEFAULT_CONFIG    "jdetnet_voc"
#define DEFAULT_INPUT     "../test/testvecs/input/horse_768x320.y"
#define DEFAULT_INPUT_FRAMES (1)
#define DEFAULT_OBJECT_CLASSES_LIST_FILE "./jdetnet_voc_objects.json"
#define DEFAULT_OUTPUT_PROB_THRESHOLD  25

/* Enable this macro to record individual output files and */
/* resized, cropped network input files                    */
//#define DEBUG_FILES

std::unique_ptr<ObjectClasses> object_classes;
uint32_t orig_width;
uint32_t orig_height;
uint32_t num_frames_file;

bool RunConfiguration(const cmdline_opts_t& opts);
Executor* CreateExecutor(DeviceType dt, uint32_t num, const Configuration& c,
                         int layers_group_id);
bool ReadFrame(ExecutionObjectPipeline& eop, uint32_t frame_idx,
               const Configuration& c, const cmdline_opts_t& opts,
               VideoCapture &cap, ifstream &ifs);
bool WriteFrameOutput(const ExecutionObjectPipeline& eop,
                      const Configuration& c, const cmdline_opts_t& opts,
                      float confidence_value);
static void DisplayHelp();

/***************************************************************/
/* Slider to control detection confidence level                */
/***************************************************************/
static void on_trackbar( int slider_id, void *inst )
{
  //This function is invoked on every slider move.
  //No action required, since prob_slider is automatically updated.
  //But, for any additional operation on slider move, this is the place to insert code.
}


int main(int argc, char *argv[])
{
    // Catch ctrl-c to ensure a clean exit
    signal(SIGABRT, exit);
    signal(SIGTERM, exit);

    // If there are no devices capable of offloading TIDL on the SoC, exit
    uint32_t num_eves = Executor::GetNumDevices(DeviceType::EVE);
    uint32_t num_dsps = Executor::GetNumDevices(DeviceType::DSP);
    if (num_eves == 0 || num_dsps == 0)
    {
        cout << "ssd_multibox requires both EVE and DSP for execution." << endl;
        return EXIT_SUCCESS;
    }

    // Process arguments
    cmdline_opts_t opts;
    opts.config = DEFAULT_CONFIG;
    opts.object_classes_list_file = DEFAULT_OBJECT_CLASSES_LIST_FILE;
    opts.num_eves = 1;
    opts.num_dsps = 1;
    opts.input_file = DEFAULT_INPUT;
    opts.output_prob_threshold = DEFAULT_OUTPUT_PROB_THRESHOLD;
    if (! ProcessArgs(argc, argv, opts))
    {
        DisplayHelp();
        exit(EXIT_SUCCESS);
    }
    assert(opts.num_dsps != 0 && opts.num_eves != 0);
    if (opts.num_frames == 0)
        opts.num_frames = (opts.is_camera_input || opts.is_video_input) ?
                          NUM_VIDEO_FRAMES :
                          ((opts.input_file == DEFAULT_INPUT) ?
                           DEFAULT_INPUT_FRAMES : 1);
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
        cout << "ssd_multibox FAILED" << endl;
        return EXIT_FAILURE;
    }

    cout << "ssd_multibox PASSED" << endl;
    return EXIT_SUCCESS;
}

bool RunConfiguration(const cmdline_opts_t& opts)
{
    int prob_slider     = opts.output_prob_threshold;
    // Read the TI DL configuration file
    Configuration c;
    std::string config_file = "../test/testvecs/config/infer/tidl_config_"
                              + opts.config + ".txt";
    bool status = c.ReadFromFile(config_file);
    if (!status)
    {
        cerr << "Error in configuration file: " << config_file << endl;
        return false;
    }
    c.enableApiTrace = opts.verbose;
    // setup camera/video input
    VideoCapture cap;
    if (! SetVideoInputOutput(cap, opts, "SSD_Multibox"))  return false;

    if (opts.is_camera_input || opts.is_video_input)
    {
        std::string TrackbarName("Confidence(%):");
        createTrackbar( TrackbarName.c_str(), "SSD_Multibox",
                        &prob_slider, 100, on_trackbar );
        std::cout << TrackbarName << std::endl;
    }

    // setup preprocessed input
    ifstream ifs;
    if (opts.is_preprocessed_input)
    {
        ifs.open(opts.input_file, ios::binary | ios::ate);
        if (! ifs.good())
        {
            cerr << "Cannot open " << opts.input_file << endl;
            return false;
        }
        num_frames_file = ((int) ifs.tellg()) /
                          (c.inWidth * c.inHeight * c.inNumChannels);
    }

    try
    {
        // Create Executors with the approriate core type, number of cores
        // and configuration specified
        // EVE will run layersGroupId 1 in the network, while
        // DSP will run layersGroupId 2 in the network
        Executor* e_eve = CreateExecutor(DeviceType::EVE, opts.num_eves, c, 1);
        Executor* e_dsp = CreateExecutor(DeviceType::DSP, opts.num_dsps, c, 2);

        // Construct ExecutionObjectPipeline that utilizes multiple
        // ExecutionObjects to process a single frame, each ExecutionObject
        // processes one layerGroup of the network
        //
        // Pipeline depth can enable more optimized pipeline execution:
        // Given one EVE and one DSP as an example, with different
        //     pipeline_depth, we have different execution behavior:
        // If pipeline_depth is set to 1,
        //    we create one EOP: eop0 (eve0, dsp0)
        //    pipeline execution of multiple frames over time is as follows:
        //    --------------------- time ------------------->
        //    eop0: [eve0...][dsp0]
        //    eop0:                [eve0...][dsp0]
        //    eop0:                               [eve0...][dsp0]
        //    eop0:                                              [eve0...][dsp0]
        // If pipeline_depth is set to 2,
        //    we create two EOPs: eop0 (eve0, dsp0), eop1(eve0, dsp0)
        //    pipeline execution of multiple frames over time is as follows:
        //    --------------------- time ------------------->
        //    eop0: [eve0...][dsp0]
        //    eop1:          [eve0...][dsp0]
        //    eop0:                   [eve0...][dsp0]
        //    eop1:                            [eve0...][dsp0]
        // Additional benefit of setting pipeline_depth to 2 is that
        //    it can also overlap host ReadFrame() with device processing:
        //    --------------------- time ------------------->
        //    eop0: [RF][eve0...][dsp0]
        //    eop1:     [RF]     [eve0...][dsp0]
        //    eop0:                    [RF][eve0...][dsp0]
        //    eop1:                             [RF][eve0...][dsp0]
        vector<ExecutionObjectPipeline *> eops;
        uint32_t pipeline_depth = 2;  // 2 EOs in EOP -> depth 2
        for (uint32_t j = 0; j < pipeline_depth; j++)
            for (uint32_t i = 0; i < max(opts.num_eves, opts.num_dsps); i++)
                eops.push_back(new ExecutionObjectPipeline(
                      {(*e_eve)[i%opts.num_eves], (*e_dsp)[i%opts.num_dsps]}));
        uint32_t num_eops = eops.size();

        // Allocate input/output memory for each EOP
        AllocateMemory(eops);

        chrono::time_point<chrono::steady_clock> tloop0, tloop1;
        tloop0 = chrono::steady_clock::now();

        // Process frames with available eops in a pipelined manner
        // additional num_eops iterations to flush pipeline (epilogue)
        for (uint32_t frame_idx = 0;
             frame_idx < opts.num_frames + num_eops; frame_idx++)
        {
            ExecutionObjectPipeline* eop = eops[frame_idx % num_eops];

            // Wait for previous frame on the same eop to finish processing
            if (eop->ProcessFrameWait())
            {
                WriteFrameOutput(*eop, c, opts, (float)prob_slider);
            }

            // Read a frame and start processing it with current eo
            if (ReadFrame(*eop, frame_idx, c, opts, cap, ifs))
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
Executor* CreateExecutor(DeviceType dt, uint32_t num, const Configuration& c,
                         int layers_group_id)
{
    if (num == 0) return nullptr;

    DeviceIds ids;
    for (uint32_t i = 0; i < num; i++)
        ids.insert(static_cast<DeviceId>(i));

    return new Executor(dt, ids, c, layers_group_id);
}

bool ReadFrame(ExecutionObjectPipeline& eop, uint32_t frame_idx,
               const Configuration& c, const cmdline_opts_t& opts,
               VideoCapture &cap, ifstream &ifs)
{
    if ((uint32_t)frame_idx >= opts.num_frames)
        return false;

    eop.SetFrameIndex(frame_idx);

    char*  frame_buffer = eop.GetInputBufferPtr();
    assert (frame_buffer != nullptr);
    int channel_size = c.inWidth * c.inHeight;
    int frame_size   = channel_size * c.inNumChannels;

    Mat image;
    if (!opts.is_camera_input && !opts.is_video_input)
    {
        if (opts.is_preprocessed_input)
        {
            orig_width  = c.inWidth;
            orig_height = c.inHeight;
            ifs.seekg((frame_idx % num_frames_file) * frame_size);
            ifs.read(frame_buffer, frame_size);
            return ifs.good();
        }
        else
        {
            image = cv::imread(opts.input_file, CV_LOAD_IMAGE_COLOR);
            if (image.empty())
            {
                cerr << "Unable to read from: " << opts.input_file << endl;
                return false;
            }
        }
    }
    else
    {
        if(opts.is_camera_input)
        {
           if (! cap.grab()) return false;
           if (! cap.retrieve(image)) return false;
        }
        else
        { // Video clip
           if (cap.grab())
           {
             if (! cap.retrieve(image)) return false;
           } else {
             //Rewind!
             std::cout << "Video clip rewinded!" << std::endl;
             cap.set(CAP_PROP_POS_FRAMES, 0);
             if (! cap.grab()) return false;
             if (! cap.retrieve(image)) return false;
           }
        }
    }

    // Scale to network input size:
    Mat s_image, bgr_frames[3];
    orig_width  = image.cols;
    orig_height = image.rows;
    if (!opts.is_camera_input && !opts.is_video_input)
    {
        cv::resize(image, s_image, Size(c.inWidth, c.inHeight),
                   0, 0, cv::INTER_AREA);
    }
    else
    {
        // Preserve aspect ratio, by doing central cropping
        // Choose vertical or horizontal central cropping
        // based on dimension reduction
        if(orig_width > orig_height)
        {
            float change_width  = (float)c.inWidth / (float)orig_width;
            float change_height = (float)c.inHeight / (float)orig_height;
            if(change_width < change_height)
            {
                // E.g. for 1920x1080->512x512, we first crop central part
                // roi(420, 0, 1080, 1080), then resize to (512x512)
                int offset_x = (int)round(0.5 * ((float)orig_width -
                  ((float)orig_height * (float)c.inWidth / (float)c.inHeight)));
                cv::resize(image(Rect(offset_x, 0, orig_width - 2 * offset_x,
                                      orig_height)), s_image,
                           Size(c.inWidth, c.inHeight), 0, 0, cv::INTER_AREA);
            } else {
                // E.g. for 1920x1080->768x320, we first crop central part
                // roi(0, 140, 1920, 800), then resize to (768x320)
                int offset_y = (int)round(0.5 * ((float)orig_height -
                  ((float)orig_width * (float)c.inHeight / (float)c.inWidth)));
                cv::resize(image(Rect(0, offset_y, orig_width,
                                      orig_height - 2 * offset_y)), s_image,
                           Size(c.inWidth, c.inHeight), 0, 0, cv::INTER_AREA);
            }
        } else {
          // E.g. for 540x960->512x512, we first crop central part
          //      roi(0, 210, 540, 540), then resize to (512x512)
          // E.g. for 540x960->768x320, we first crop central part
          //      roi(0, 367, 540, 225), then resize to (768x320)
          int offset_y = (int)round(0.5 * ((float)orig_height -
              ((float)orig_width * (float)c.inHeight / (float)c.inWidth)));
          cv::resize(image(Rect(0, offset_y, orig_width, orig_height -
                                2 * offset_y)), s_image,
                     Size(c.inWidth, c.inHeight), 0, 0, cv::INTER_AREA);
        }
    }

    #ifdef DEBUG_FILES
    {
        // Image files can be converted into video using, example script
        // (on desktop Ubuntu, with ffmpeg installed):
        // ffmpeg -i netin_%04d.png -vf "scale=(iw*sar)*max(768/(iw*sar)\,320/ih):ih*max(768/(iw*sar)\,320/ih), crop=768:320" -b:v 4000k out.mp4
        // Update width 768, height 320, if necessary
        char netin_name[80];
        sprintf(netin_name, "netin_%04d.png", frame_idx);
        cv::imwrite(netin_name, s_image);
        std::cout << "Video input, width:" << orig_width << " height:"
                  << orig_height << " Network width:" << c.inWidth
                  << " height:" << c.inHeight << std::endl;
    }
    #endif

    cv::split(s_image, bgr_frames);
    memcpy(frame_buffer,                bgr_frames[0].ptr(), channel_size);
    memcpy(frame_buffer+1*channel_size, bgr_frames[1].ptr(), channel_size);
    memcpy(frame_buffer+2*channel_size, bgr_frames[2].ptr(), channel_size);
    return true;
}

// Create frame with boxes drawn around classified objects
bool WriteFrameOutput(const ExecutionObjectPipeline& eop,
                      const Configuration& c, const cmdline_opts_t& opts,
                      float confidence_value)
{
    // Asseembly original frame
    int width  = c.inWidth;
    int height = c.inHeight;
    int channel_size = width * height;
    Mat frame, bgr[3];

    unsigned char *in = (unsigned char *) eop.GetInputBufferPtr();
    bgr[0] = Mat(height, width, CV_8UC(1), in);
    bgr[1] = Mat(height, width, CV_8UC(1), in + channel_size);
    bgr[2] = Mat(height, width, CV_8UC(1), in + channel_size*2);
    cv::merge(bgr, 3, frame);

    int frame_index = eop.GetFrameIndex();
    char outfile_name[64];
    if (opts.is_preprocessed_input)
    {
        snprintf(outfile_name, 64, "frame_%d.png", frame_index);
        cv::imwrite(outfile_name, frame);
        printf("Saving frame %d to: %s\n", frame_index, outfile_name);
    }

    // Draw boxes around classified objects
    float *out = (float *) eop.GetOutputBufferPtr();
    int num_floats = eop.GetOutputBufferSizeInBytes() / sizeof(float);
    for (int i = 0; i < num_floats / 7; i++)
    {
        int index = (int)    out[i * 7 + 0];
        if (index < 0)  break;

        float score =        out[i * 7 + 2];
        if (score * 100 < confidence_value)  continue;

        int   label = (int)  out[i * 7 + 1];
        int   xmin  = (int) (out[i * 7 + 3] * width);
        int   ymin  = (int) (out[i * 7 + 4] * height);
        int   xmax  = (int) (out[i * 7 + 5] * width);
        int   ymax  = (int) (out[i * 7 + 6] * height);

        const ObjectClass& object_class = object_classes->At(label);

        if(opts.verbose) {
            printf("%2d: (%d, %d) -> (%d, %d): %s, score=%f\n",
               i, xmin, ymin, xmax, ymax, object_class.label.c_str(), score);
        }

        if (xmin < 0)       xmin = 0;
        if (ymin < 0)       ymin = 0;
        if (xmax > width)   xmax = width;
        if (ymax > height)  ymax = height;
        cv::rectangle(frame, Point(xmin, ymin), Point(xmax, ymax),
                      Scalar(object_class.color.blue,
                             object_class.color.green,
                             object_class.color.red), 2);
    }

    if (opts.is_camera_input || opts.is_video_input)
    {
        cv::imshow("SSD_Multibox", frame);
#ifdef DEBUG_FILES
        // Image files can be converted into video using, example script
        // (on desktop Ubuntu, with ffmpeg installed):
        // ffmpeg -i multibox_%04d.png -vf "scale=(iw*sar)*max(768/(iw*sar)\,320/ih):ih*max(768/(iw*sar)\,320/ih), crop=768:320" -b:v 4000k out.mp4
        // Update width 768, height 320, if necessary
        snprintf(outfile_name, 64, "multibox_%04d.png", frame_index);
        cv::imwrite(outfile_name, r_frame);
#endif
        waitKey(1);
    }
    else
    {
        // Resize to output width/height, keep aspect ratio
        Mat r_frame;
        uint32_t output_width = opts.output_width;
        if (output_width == 0)  output_width = orig_width;
        uint32_t output_height = (output_width*1.0f) / orig_width * orig_height;
        cv::resize(frame, r_frame, Size(output_width, output_height));

        snprintf(outfile_name, 64, "multibox_%d.png", frame_index);
        cv::imwrite(outfile_name, frame);
        printf("Saving frame %d with SSD multiboxes to: %s\n",
               frame_index, outfile_name);
    }

    return true;
}

void DisplayHelp()
{
    std::cout <<
    "Usage: ssd_multibox\n"
    "  Will run partitioned ssd_multibox network to perform "
    "multi-objects detection\n"
    "  and classification.  First part of network "
    "(layersGroupId 1) runs on EVE,\n"
    "  second part (layersGroupId 2) runs on DSP.\n"
    "  Use -c to run a different segmentation network.  Default is jdetnet_voc.\n"
    "Optional arguments:\n"
    " -c <config>          Valid configs: jdetnet_voc, jdetnet \n"
    " -d <number>          Number of dsp cores to use\n"
    " -e <number>          Number of eve cores to use\n"
    " -i <image>           Path to the image file as input\n"
    "                      Default are 9 frames in testvecs\n"
    " -i camera<number>    Use camera as input\n"
    "                      video input port: /dev/video<number>\n"
    " -i <name>.{mp4,mov,avi}  Use video file as input\n"
    " -l <objects_list>    Path to the object classes list file\n"
    " -f <number>          Number of frames to process\n"
    " -w <number>          Output image/video width\n"
    " -p <number>          Output probability threshold in percentage\n"
    "                      Default is 25 percent or higher\n"
    " -v                   Verbose output during execution\n"
    " -h                   Help\n";
}

