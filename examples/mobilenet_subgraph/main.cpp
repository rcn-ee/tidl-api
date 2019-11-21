/******************************************************************************
 * Copyright (c) 2019, Texas Instruments Incorporated - http://www.ti.com/
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
#include <future>

#include "executor.h"
#include "execution_object.h"
#include "execution_object_pipeline.h"
#include "subgraph_runtime.h"
#include "subgraph_data_conv.h"
#include "configuration.h"
#include "../common/object_classes.h"
#include "imgutil.h"
#include "../common/video_utils.h"
#include "thread_pool.h"

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
typedef struct {
  float **inputs;
  float **outputs;
} UserData;

bool RunConfiguration(cmdline_opts_t& opts);
bool ReadFrame(const cmdline_opts_t& opts, VideoCapture &cap, float** inputs,
               int batch_size);
bool WriteFrameOutput(float *out, const cmdline_opts_t& opts);
void DisplayHelp();
void SubgraphUserFunc(void *user_data);

const int num_printed_outputs = 4;
bool SkipOutputs(int i, int offset, bool &skip_outputs)
{
    if (skip_outputs)  return true;
    if (i >= num_printed_outputs + offset)
    {
        if (! skip_outputs)
        {
            cout << "   ... skippping outputs ..." << endl;
            skip_outputs = true;
        }
    }
    return skip_outputs;
}

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
    bool status = true;

    // setup camera/video input/output
    VideoCapture cap;
    if (! SetVideoInputOutput(cap, opts, "ImageNet"))  return false;

    cout << "\n##### Batch size 1 testing ######\n" << endl;
    try
    {
        TidlInitSubgraph(1, 0);
        float **inputs = new float *[1];
        inputs[0] = new float[1*3*224*224];
        float **outputs = new float *[1];
        outputs[0] = new float[1001];

        for (int i = 0; i < 5; i ++)
        {
          chrono::time_point<chrono::steady_clock> tloop0, tloop1;
          tloop0 = chrono::steady_clock::now();

          ReadFrame(opts, cap, inputs, 1);
          TidlRunSubgraph(1, 0, 1, 1, 1, inputs, outputs);
          WriteFrameOutput(outputs[0], opts);

          tloop1 = chrono::steady_clock::now();
          chrono::duration<float> elapsed = tloop1 - tloop0;
          cout << "Frame " << i
               << " time (including read/write/opencv/print/etc): "
               << setw(6) << setprecision(4)
               << (elapsed.count() * 1000) << "ms" << endl;
        }

        delete [] inputs[0];
        delete [] inputs;
        delete [] outputs[0];
        delete [] outputs;
    }
    catch (tidl::Exception &e)
    {
        cerr << e.what() << endl;
        status = false;
    }

    // If not doing multi-threaded processing, multiply by 2 or more
    //     for a larger batch to amortize batch initilization/tear down cost
    int preferred_batch_size = TidlGetPreferredBatchSize(1);
    for (int multiple = 1; multiple <= 16; multiple *= 2)
    {
        int batch_size = preferred_batch_size * multiple;
        cout << "\n##### Batch size " << batch_size << " testing ######\n"
             << endl;
        bool skip_outputs = false;
        try
        {
            float **inputs  = new float *[batch_size];
            float **outputs = new float *[batch_size];
            for (int i = 0; i < batch_size; i++)
            {
                inputs[i]  = new float[1*3*224*224];
                outputs[i] = new float[1001];
            }

            chrono::time_point<chrono::steady_clock> tloop0, tloop1;
            tloop0 = chrono::steady_clock::now();

            ReadFrame(opts, cap, inputs, batch_size);
            TidlRunSubgraph(1, 0, batch_size, 1, 1, inputs, outputs);
            for (int i = 0; i < batch_size; i++)
            {
                if (! SkipOutputs(i, 0, skip_outputs))
                {
                    cout << "Frame " << i << " of " << batch_size
                         << " output:" << endl;
                    WriteFrameOutput(outputs[i], opts);
                }
            }

            tloop1 = chrono::steady_clock::now();
            chrono::duration<float> elapsed = tloop1 - tloop0;
            cout << "Batch size " << batch_size
                 << " time: "
                 << setw(6) << setprecision(4)
                 << (elapsed.count() * 1000) << "ms, fps = "
                 << setw(6) << setprecision(4)
                 << (batch_size / elapsed.count())
                 << endl;

            for (int i = 0; i < batch_size; i++)
            {
                delete [] inputs[i];
                delete [] outputs[i];
            }
            delete [] inputs;
            delete [] outputs;
        }
        catch (tidl::Exception &e)
        {
            cerr << e.what() << endl;
            status = false;
        }
    }

    // This is to test the multithreaded inference with async/future
    // async/future has slightly worse threading performance than
    //     thread pool, however, it is much easier to program
    cout << "\n##### Multithreaded inference testing (async/future) #####\n"
         << endl;
    int num_threads = TidlGetPreferredBatchSize(1) * 2;
    int num_iters = 100;
    try
    {
        float **inputs  = new float *[num_threads];
        float **outputs = new float *[num_threads];
        for (int i = 0; i < num_threads; i++)
        {
            inputs[i]  = new float[1*3*224*224];
            outputs[i] = new float[1001];
        }
        vector<future<bool>> futures(num_threads);
        bool skip_outputs = false;

        chrono::time_point<chrono::steady_clock> tloop0, tloop1;
        tloop0 = chrono::steady_clock::now();

        for (int i = 0; i < num_iters + num_threads; i++)
        {
            int index = i % num_threads;
            if (i >= num_threads)
            {
                if (futures[index].get())
                {
                    if (! SkipOutputs(i, num_threads, skip_outputs))
                        WriteFrameOutput(outputs[index], opts);
                }
            }

            if (i < num_iters)
            {
                ReadFrame(opts, cap, &inputs[index], 1);
                futures[index] = std::async(std::launch::async,
                              [inputs, outputs](int index) {
                                  TidlRunSubgraph(1, 0, 1, 1, 1,
                                              &inputs[index], &outputs[index]);
                                   return true;
                              },
                                            index);
            }
        }

        tloop1 = chrono::steady_clock::now();
        chrono::duration<float> elapsed = tloop1 - tloop0;
        cout << "Multithreaded (num_threads=" << num_threads
             << ", batch_size=1) loop time (" << num_iters << " frames): "
             << setw(6) << setprecision(4)
             << (elapsed.count() * 1000) << "ms, fps = "
             << setw(6) << setprecision(4)
             << (num_iters / elapsed.count())
             << endl;

        for (int i = 0; i < num_threads; i++)
        {
            delete [] inputs[i];
            delete [] outputs[i];
        }
        delete [] inputs;
        delete [] outputs;
    }
    catch (tidl::Exception &e)
    {
        cerr << e.what() << endl;
        status = false;
    }

    // This is to test the multithreaded inference with a thread pool
    cout << "\n##### Multithreaded inference testing (thread pool) #####\n"
         << endl;
    try
    {
        float **inputs  = new float *[num_threads];
        float **outputs = new float *[num_threads];
        vector<UserData> v_data(num_threads);
        for (int i = 0; i < num_threads; i++)
        {
            inputs[i]  = new float[1*3*224*224];
            outputs[i] = new float[1001];
            v_data[i].inputs  = &inputs[i];
            v_data[i].outputs = &outputs[i];
        }
        ThPool pool(num_threads, SubgraphUserFunc);
        vector<int> th_ids(num_threads);
        bool skip_outputs = false;

        chrono::time_point<chrono::steady_clock> tloop0, tloop1;
        tloop0 = chrono::steady_clock::now();

        for (int i = 0; i < num_iters + num_threads; i++)
        {
            int index = i % num_threads;
            if (i >= num_threads)
            {
                UserData *data = (UserData *) pool.Wait(th_ids[index]);
                if (! SkipOutputs(i, num_threads, skip_outputs))
                    WriteFrameOutput(data->outputs[0], opts);
            }

            if (i < num_iters)
            {
                ReadFrame(opts, cap, &inputs[index], 1);
                th_ids[index] = pool.RunAsync(&v_data[index]);
            }
        }

        tloop1 = chrono::steady_clock::now();
        chrono::duration<float> elapsed = tloop1 - tloop0;
        cout << "Multithreaded (num_threads=" << num_threads
             << ", batch_size=1) loop time (" << num_iters << " frames): "
             << setw(6) << setprecision(4)
             << (elapsed.count() * 1000) << "ms, fps = "
             << setw(6) << setprecision(4)
             << (num_iters / elapsed.count())
             << endl;

        for (int i = 0; i < num_threads; i++)
        {
            delete [] inputs[i];
            delete [] outputs[i];
        }
        delete [] inputs;
        delete [] outputs;
    }
    catch (tidl::Exception &e)
    {
        cerr << e.what() << endl;
        status = false;
    }

    num_threads = 2;
    int batch_size  = preferred_batch_size;
    // This is to test the multithreaded batch inference with async/future
    // Ideally, batch_size * num_threads <= number of threads
    cout << "\n##### Multithreaded batch inference testing (async/future)"
         << " #####\n" << endl;
    try
    {
        float **inputs  = new float *[num_threads * batch_size];
        float **outputs = new float *[num_threads * batch_size];
        for (int i = 0; i < num_threads * batch_size; i++)
        {
            inputs[i]  = new float[1*3*224*224];
            outputs[i] = new float[1001];
        }
        vector<future<bool>> futures(num_threads);
        bool skip_outputs = false;

        chrono::time_point<chrono::steady_clock> tloop0, tloop1;
        tloop0 = chrono::steady_clock::now();

        for (int i = 0; i < num_iters/batch_size + num_threads; i++)
        {
            int index = i % num_threads;
            if (i >= num_threads)
            {
                if (futures[index].get())
                    if (! SkipOutputs(i*batch_size, num_threads*batch_size,
                                      skip_outputs))
                        for (int b = 0; b < batch_size; b++)
                            WriteFrameOutput(outputs[index*batch_size+b], opts);
            }

            if (i < num_iters/batch_size)
            {
                ReadFrame(opts, cap, &inputs[index*batch_size], batch_size);
                futures[index] = std::async(std::launch::async,
                      [inputs, outputs, batch_size](int index) {
                          TidlRunSubgraph(1, 0, batch_size, 1, 1,
                                          &inputs[index*batch_size],
                                          &outputs[index*batch_size]);
                          return true;
                      },
                                            index);
            }
        }

        tloop1 = chrono::steady_clock::now();
        chrono::duration<float> elapsed = tloop1 - tloop0;
        cout << "Multithreaded batch (num_threads=" << num_threads
             << ", batch_size=" << batch_size
             << ") loop time (" << num_iters << " frames): "
             << setw(6) << setprecision(4)
             << (elapsed.count() * 1000) << "ms, fps = "
             << setw(6) << setprecision(4)
             << (num_iters / elapsed.count())
             << endl;

        for (int i = 0; i < num_threads * batch_size; i++)
        {
            delete [] inputs[i];
            delete [] outputs[i];
        }
        delete [] inputs;
        delete [] outputs;
    }
    catch (tidl::Exception &e)
    {
        cerr << e.what() << endl;
        status = false;
    }


    return status;
}

void SubgraphUserFunc(void *user_data)
{
  UserData *data = (UserData *) user_data;
  //printf("data inputs = %p, outputs = %p\n", data->inputs, data->outputs);
  TidlRunSubgraph(1, 0, 1, 1, 1, data->inputs, data->outputs);
  //printf("TidlRunSubgraph finished\n");
}

bool ReadFrame(const cmdline_opts_t& opts, VideoCapture &cap, float** inputs,
               int batch_size)
{
    Configuration c;
    c.inNumChannels = 3;;
    c.inWidth = 224;
    c.inHeight = 224;
    c.preProcType = 2;
    SubgraphDataConv in_conv{{0}, {true}, {128.0f}, {false}, {1,3,224,224}};

    char* frame_buffer = new char[3*224*224];
    assert (frame_buffer != nullptr);

    Mat image;
    if (! opts.is_camera_input && ! opts.is_video_input)
    {
        if (opts.input_file.empty())
            image = cv::imread(default_inputs[0],
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
    bool status = imgutil::PreprocessImage(image, frame_buffer, c);
    for (int i = 0; i < batch_size; i++)
    {
        std::vector<float *> in_data_v{inputs[i]};
        in_conv.ScaleDequant((const uint8_t *)frame_buffer, in_data_v);
    }
    delete [] frame_buffer;
    return status;
}

// Display top 5 classified imagenet classes with probabilities 5% or higher
bool WriteFrameOutput(float *out, const cmdline_opts_t& opts)
{
    const int k = 5;
    int out_size =  1001;
    // Tensorflow trained network outputs 1001 probabilities,
    // with 0-index being background, thus we need to subtract 1 when
    // reporting classified object from 1000 categories
    int background_offset = out_size == 1001 ? 1 : 0;

    // sort and get k largest values and corresponding indices
    typedef pair<float, int> val_index;
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

    for (int i = k - 1; i >= 0; i--)
    {
        if (sorted[i].first * 100 < opts.output_prob_threshold)  break;
        int imagenet_index = sorted[i].second - background_offset;
        cout << k-i << ": [" << imagenet_index << "] "
             << object_classes->At(imagenet_index).label
             << ",   prob = " << setprecision(4)
             << (sorted[i].first * 100) << "%" << endl;
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

