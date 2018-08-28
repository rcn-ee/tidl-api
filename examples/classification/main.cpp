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
#include <queue>
#include <algorithm>
#include <time.h>
#include <memory.h>
#include <string.h>

#include "executor.h"
#include "execution_object.h"
#include "execution_object_pipeline.h"
#include "configuration.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

//#define TWO_ROIs
#define LIVE_DISPLAY
#define PERF_VERBOSE
//#define RMT_GST_STREAMER

#define MAX_NUM_ROI 4

int live_input = 1;
char video_clip[320];

#ifdef TWO_ROIs
#define RES_X 400
#define RES_Y 300
#define NUM_ROI_X 2
#define NUM_ROI_Y 1
#define X_OFFSET 0
#define X_STEP   176
#define Y_OFFSET 52
#define Y_STEP   224
#else
#define RES_X 244
#define RES_Y 244
#define NUM_ROI_X 1
#define NUM_ROI_Y 1
#define X_OFFSET 10
#define X_STEP   224
#define Y_OFFSET 10
#define Y_STEP   224
#endif

int NUM_ROI = NUM_ROI_X * NUM_ROI_Y;

//Temporal averaging
int TOP_CANDIDATES = 3;

using namespace tidl;
using namespace cv;

#ifdef LIVE_DISPLAY
void imagenetCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_RBUTTONDOWN )
    {
        std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << " ... prepare to exit!" << std::endl;
        exit(0);
    }
}
#endif

static int tf_postprocess(uchar *in, int size, int roi_idx, int frame_idx, int f_id);
static void tf_preprocess(uchar *out, uchar *in, int size);
static int ShowRegion(int roi_history[]);
static int selclass_history[MAX_NUM_ROI][3];  // from most recent to oldest at top indices

bool __TI_show_debug_ = false;

bool RunConfiguration(const std::string& config_file, int num_layers_groups,
                      uint32_t num_dsps, uint32_t num_eves);

bool ReadFrame(ExecutionObjectPipeline&   ep,
               int                  frame_idx,
               const Configuration& configuration,
               std::istream&        input_file);

static void ProcessArgs(int argc, char *argv[],
                        std::string& config_file,
                        uint32_t & num_dsps, uint32_t &num_eves,
                        int & num_layers_groups);

static void DisplayHelp();
extern std::string labels_classes[];
extern int IMAGE_CLASSES_NUM;
extern int selected_items_size;
extern int selected_items[];
extern int populate_selected_items (char *filename);
extern void populate_labels (char *filename);

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

int main(int argc, char *argv[])
{
    // Catch ctrl-c to ensure a clean exit
    signal(SIGABRT, exit);
    signal(SIGTERM, exit);

    // If there are no devices capable of offloading TIDL on the SoC, exit
    uint32_t num_eves = Executor::GetNumDevices(DeviceType::EVE);
    uint32_t num_dsps = Executor::GetNumDevices(DeviceType::DSP);
    int num_layers_groups = 1;

    if (num_eves == 0 && num_dsps == 0)
    {
        std::cout << "TI DL not supported on this SoC." << std::endl;
        return EXIT_SUCCESS;
    }

    // Process arguments
    std::string config_file;
    ProcessArgs(argc, argv, config_file, num_dsps, num_eves, num_layers_groups);

    bool status = true;
    if (!config_file.empty()) {
        std::cout << "Run single configuration: " << config_file << std::endl;
        status = RunConfiguration(config_file, num_layers_groups, num_dsps, num_eves);
    } else
    {
        status = false;
    }

    if (!status)
    {
        std::cout << "tidl FAILED" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "tidl PASSED" << std::endl;
    return EXIT_SUCCESS;
}

bool RunConfiguration(const std::string& config_file, int num_layers_groups, uint32_t num_dsps, uint32_t num_eves)
{
    char imagenet_win[160];

    DeviceIds ids_eve, ids_dsp;

    for (uint32_t i = 0; i < num_eves; i++)
        ids_eve.insert(static_cast<DeviceId>(i));
    for (uint32_t i = 0; i < num_dsps; i++)
        ids_dsp.insert(static_cast<DeviceId>(i));


    // Read the TI DL configuration file
    Configuration configuration;
    bool status = configuration.ReadFromFile(config_file);
    if (!status)
    {
        std::cerr << "Error in configuration file: " << config_file
                  << std::endl;
        return false;
    }

    std::ifstream input_data_file(configuration.inData, std::ios::binary);
    std::ofstream output_data_file(configuration.outData, std::ios::binary);
    assert (input_data_file.good());
    assert (output_data_file.good());

    sprintf(imagenet_win, "Imagenet_EVEx%d_DSPx%d", num_eves, num_dsps);

    try
    {
        Executor *exe_eve = NULL;
        Executor *exe_dsp = NULL;
        int num_eps = 0;
        std::vector<ExecutionObjectPipeline *> eps;
        switch(num_layers_groups)
        {
        case 1: // Single layers group
           exe_eve = num_eves > 0 ? new Executor(DeviceType::EVE, ids_eve, configuration) : NULL;
           exe_dsp = num_dsps > 0 ? new Executor(DeviceType::DSP, ids_dsp, configuration) : NULL;
           num_eps = num_eves + num_dsps;

           // Construct ExecutionObjectPipeline with single Execution Object to
           // process each frame. This is parallel processing of frames with as many
           // DSP and EVE cores that we have on hand.
           for (uint32_t i = 0; i < num_eves; i++)
              eps.push_back(new ExecutionObjectPipeline({(*exe_eve)[i]}));

           for (uint32_t i = 0; i < num_dsps; i++)
              eps.push_back(new ExecutionObjectPipeline({(*exe_dsp)[i]}));

           break;

         case 2: // Two layers group
          // JacintoNet11 specific : specify only layers that will be in layers group 2
          // ...by default all other layers are in group 1.
          configuration.layerIndex2LayerGroupId = { {12, 2}, {13, 2}, {14, 2} };

          // Create a executor with the approriate core type, number of cores
          // and configuration specified
          // EVE will run layersGroupId 1 in the network, while
          // DSP will run layersGroupId 2 in the network
          exe_eve = num_eves > 0 ? new Executor(DeviceType::EVE, ids_eve, configuration, 1) : NULL;
          exe_dsp = num_dsps > 0 ? new Executor(DeviceType::DSP, ids_dsp, configuration, 2) : NULL;

          // Construct ExecutionObjectPipeline that utilizes multiple
          // ExecutionObjects to process a single frame, each ExecutionObject
          // processes one layerGroup of the network
          num_eps = std::max(num_eves, num_dsps);
          for (int i = 0; i < num_eps; i++)
              eps.push_back(new ExecutionObjectPipeline({(*exe_eve)[i%num_eves],
                                                         (*exe_dsp)[i%num_dsps]}));
          break;

         default:
           std::cout << "Layers groups can be either 1 or 2!" << std::endl;
           return false;
        }
        // Allocate input/output memory for each EOP
        std::vector<void *> buffers;
        for (auto ep : eps)
        {
           size_t in_size  = ep->GetInputBufferSizeInBytes();
           size_t out_size = ep->GetOutputBufferSizeInBytes();
           void*  in_ptr   = malloc(in_size);
           void*  out_ptr  = malloc(out_size);
           assert(in_ptr != nullptr && out_ptr != nullptr);
           buffers.push_back(in_ptr);
           buffers.push_back(out_ptr);

           ArgInfo in(in_ptr,   in_size);
           ArgInfo out(out_ptr, out_size);
           ep->SetInputOutputBuffer(in, out);
        }

#ifdef LIVE_DISPLAY
    if(NUM_ROI > 1)
    {
      for(int i = 0; i < NUM_ROI; i ++) {
        char tmp_string[80];
        sprintf(tmp_string, "ROI[%02d]", i);
        namedWindow(tmp_string, WINDOW_AUTOSIZE | CV_GUI_NORMAL);
      }
    }
    Mat sw_stack_image = imread("/usr/share/ti/tidl/examples/classification/tidl-sw-stack-small.png", IMREAD_COLOR); // Read the file
    if( sw_stack_image.empty() )                      // Check for invalid input
    {
      std::cout <<  "Could not open or find the tidl-sw-stack-small image" << std::endl ;
    } else {
      namedWindow( "TIDL SW Stack", WINDOW_AUTOSIZE | CV_GUI_NORMAL ); // Create a window for display.
      cv::imshow( "TIDL SW Stack", sw_stack_image );                // Show our image inside it.
    }

    namedWindow("ClassList", WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    namedWindow(imagenet_win, WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    //set the callback function for any mouse event
    setMouseCallback(imagenet_win, imagenetCallBackFunc, NULL);

    Mat classlist_image = cv::Mat::zeros(40 + selected_items_size * 20, 220, CV_8UC3);
    char tmp_classwindow_string[160];
    //Erase window
    classlist_image.setTo(Scalar::all(0));

    for (int i = 0; i < selected_items_size; i ++)
    {
      sprintf(tmp_classwindow_string, "%2d) %12s", 1+i, labels_classes[selected_items[i]].c_str());
      cv::putText(classlist_image, tmp_classwindow_string,
                  cv::Point(5, 40 + i * 20),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  0.75,
                  cv::Scalar(255,255,255), 1, 8);
    }
    cv::imshow("ClassList", classlist_image);

#endif
    Mat r_frame, r_mframe, r_blend;
    Mat to_stream;
    VideoCapture cap;
    double avg_fps = 0.0;

   if(live_input >= 0)
   {
      cap.open(live_input);
      VideoWriter writer;  // gstreamer

      const double fps = cap.get(CAP_PROP_FPS);
      const int width  = cap.get(CAP_PROP_FRAME_WIDTH);
      const int height = cap.get(CAP_PROP_FRAME_HEIGHT);
      std::cout << "Capture camera with " << fps << " fps, " << width << "x" << height << " px" << std::endl;

#ifdef RMT_GST_STREAMER
      writer.open(" appsrc ! videoconvert ! video/x-raw, format=(string)NV12, width=(int)640, height=(int)480, framerate=(fraction)30/1 ! \
                ducatih264enc bitrate=2000 ! queue ! h264parse config-interval=1 ! \
                mpegtsmux ! udpsink host=192.168.1.2 sync=false port=5000",
                0,fps,Size(640,480),true);

      if (!writer.isOpened()) {
        cap.release();
        std::cerr << "Can't create gstreamer writer. Do you have the correct version installed?" << std::endl;
        std::cerr << "Print out OpenCV build information" << std::endl;
        std::cout << getBuildInformation() << std::endl;
        return false;
      }
#endif
   } else {
     std::cout << "Video input clip: " << video_clip << std::endl;
     cap.open(std::string(video_clip));
      const double fps = cap.get(CAP_PROP_FPS);
      const int width  = cap.get(CAP_PROP_FRAME_WIDTH);
      const int height = cap.get(CAP_PROP_FRAME_HEIGHT);
      std::cout << "Clip with " << fps << " fps, " << width << "x" << height << " px" << std::endl;

   }
   std::cout << "About to start ProcessFrame loop!!" << std::endl;


   Rect rectCrop[NUM_ROI];
   for (int y = 0; y < NUM_ROI_Y; y ++) {
      for (int x = 0; x < NUM_ROI_X; x ++) {
         rectCrop[y * NUM_ROI_X + x] = Rect(X_OFFSET + x * X_STEP, Y_OFFSET + y * Y_STEP, 224, 224);
         std::cout << "Rect[" << X_OFFSET + x * X_STEP << ", " << Y_OFFSET + y * Y_STEP << "]" << std::endl;
      }
   }
   int num_frames = 99999;

   if (!cap.isOpened()) {
      std::cout << "Video input not opened!" << std::endl;
      return false;
   }
   Mat in_image, image, r_image, show_image, bgr_frames[3];
   int is_object;
   for(int k = 0; k < NUM_ROI; k++) {
      for(int i = 0; i < 3; i ++) selclass_history[k][i] = -1;
   }

   // Process frames with available execution objects in a pipelined manner
   // additional num_eps iterations to flush the pipeline (epilogue)
   for (int frame_idx = 0; frame_idx < configuration.numFrames + num_eps; frame_idx++)
   {
        ExecutionObjectPipeline* ep = eps[frame_idx % num_eps];

        // Wait for previous frame on the same eo to finish processing
        if (ep->ProcessFrameWait())
        {
             double elapsed_host = ep->GetHostProcessTimeInMilliSeconds();
             /* Exponential averaging */
             avg_fps = 0.1 * ((double)num_eps * 1000.0 / ((double)NUM_ROI * elapsed_host)) + 0.9 * avg_fps;
#ifdef PERF_VERBOSE
             ReportTime(ep->GetFrameIndex(), ep->GetDeviceName(),
                        ep->GetHostProcessTimeInMilliSeconds(),
                        ep->GetProcessTimeInMilliSeconds());
#endif
             int f_id = ep->GetFrameIndex();
             int curr_roi = f_id % NUM_ROI;
             is_object = tf_postprocess((uchar*) ep->GetOutputBufferPtr(), IMAGE_CLASSES_NUM, curr_roi, frame_idx, f_id);
             selclass_history[curr_roi][2] = selclass_history[curr_roi][1];
             selclass_history[curr_roi][1] = selclass_history[curr_roi][0];
             selclass_history[curr_roi][0] = is_object;
             for (int r = 0; r < NUM_ROI; r ++)
             {
	        int rpt_id =  ShowRegion(selclass_history[r]);
                if(rpt_id >= 0)
                {
                  // overlay the display window, if ball seen during last two times
                  cv::putText(show_image, labels_classes[rpt_id].c_str(),
                    cv::Point(rectCrop[r].x + 5,rectCrop[r].y + 20), // Coordinates
                    cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                    1.0, // Scale. 2.0 = 2x bigger
                    cv::Scalar(0,0,255), // Color
                    1, // Thickness
                    8); // Line type
                  cv::rectangle(show_image, rectCrop[r], Scalar(255,0,0), 3);
                  std::cout << "ROI(" << r << ")(" << rpt_id << ")=" << labels_classes[rpt_id].c_str() << std::endl;

                  classlist_image.setTo(Scalar::all(0));
                  for (int k = 0; k < selected_items_size; k ++)
                  {
                     sprintf(tmp_classwindow_string, "%2d) %12s", 1+k, labels_classes[selected_items[k]].c_str());
                     cv::putText(classlist_image, tmp_classwindow_string,
                                 cv::Point(5, 40 + k * 20),
                                 cv::FONT_HERSHEY_COMPLEX_SMALL,
                                 0.75,
                                 selected_items[k] == rpt_id ? cv::Scalar(0,0,255) : cv::Scalar(255,255,255), 1, 8);
                  }
                  sprintf(tmp_classwindow_string, "FPS:%5.2lf", avg_fps );

#ifdef PERF_VERBOSE
                  std::cout << "Device:" << ep->GetDeviceName() << " eps(" << num_eps << "), EVES(" << num_eves <<
                       ") DSPS(" << num_dsps << ") FPS:" << avg_fps << std::endl;
#endif
                  cv::putText(classlist_image, tmp_classwindow_string,
                              cv::Point(5, 20),
                              cv::FONT_HERSHEY_COMPLEX_SMALL,
                              0.75,
                              cv::Scalar(0,255,0), 1, 8);
                  cv::imshow("ClassList", classlist_image);
               }
             }
#ifdef LIVE_DISPLAY
             cv::imshow(imagenet_win, show_image);
#endif

#ifdef RMT_GST_STREAMER
             cv::resize(show_image, to_stream, cv::Size(640,480));
             writer << to_stream;
#endif

#ifdef LIVE_DISPLAY
             waitKey(2);
#endif
        }

        if (cap.grab() && frame_idx < num_frames)
        {
            if (cap.retrieve(in_image))
            {
                cv::resize(in_image, image, Size(RES_X,RES_Y));
                r_image = Mat(image, rectCrop[frame_idx % NUM_ROI]);

#ifdef LIVE_DISPLAY
                if(NUM_ROI > 1)
                {
                   char tmp_string[80];
                   sprintf(tmp_string, "ROI[%02d]", frame_idx % NUM_ROI);
                   cv::imshow(tmp_string, r_image);
                }
#endif
                //Convert from BGR pixel interleaved to BGR plane interleaved!
                cv::split(r_image, bgr_frames);
                tf_preprocess((uchar*) ep->GetInputBufferPtr(), bgr_frames[0].ptr(), 224*224);
                tf_preprocess((uchar*) ep->GetInputBufferPtr()+224*224, bgr_frames[1].ptr(), 224*224);
                tf_preprocess((uchar*) ep->GetInputBufferPtr()+2*224*224, bgr_frames[2].ptr(), 224*224);
                ep->SetFrameIndex(frame_idx);
                ep->ProcessFrameStartAsync();

#ifdef RMT_GST_STREAMER
                cv::resize(Mat(image, Rect(0,32,640,448)), to_stream, Size(640,480));
                writer << to_stream;
#endif

#ifdef LIVE_DISPLAY
                //waitKey(2);
                image.copyTo(show_image);
#endif
            }
        } else {
          if(live_input == -1) {
            //Rewind!
            cap.release();
            cap.open(std::string(video_clip));
          }
        }
      }
      for (auto ep : eps)    delete ep;
      for (auto b : buffers) free(b);
      if(num_dsps) delete exe_dsp;
      if(num_eves) delete exe_eve;
    }
    catch (tidl::Exception &e)
    {
        std::cerr << e.what() << std::endl;
        status = false;
    }


    input_data_file.close();
    output_data_file.close();

    return status;
}

bool ReadFrame(ExecutionObjectPipeline &ep, int frame_idx,
               const Configuration& configuration,
               std::istream& input_file)
{
    if (frame_idx >= configuration.numFrames)
        return false;

    char*  frame_buffer = ep.GetInputBufferPtr();
    assert (frame_buffer != nullptr);

    memset (frame_buffer, 0,  ep.GetInputBufferSizeInBytes());
    input_file.read(frame_buffer, ep.GetInputBufferSizeInBytes() / (configuration.inNumChannels == 1 ? 2 : 1));

    if (input_file.eof())
        return false;

    assert (input_file.good());

    // Set the frame index  being processed by the EO. This is used to
    // sort the frames before they are output
    ep.SetFrameIndex(frame_idx);

    if (input_file.good())
        return true;

    return false;
}

// Function to process all command line arguments
void ProcessArgs(int argc, char *argv[], std::string& config_file,
                 uint32_t & num_dsps, uint32_t & num_eves, int & num_layers_groups )
{
    const struct option long_options[] =
    {
        {"labels_classes_file", required_argument, 0, 'l'},
        {"selected_classes_file", required_argument, 0, 's'},
        {"config_file", required_argument, 0, 'c'},
        {"num_dsps", required_argument, 0, 'd'},
        {"num_eves", required_argument, 0, 'e'},
        {"num_layers_groups", required_argument, 0, 'g'},
        {"help",        no_argument,       0, 'h'},
        {"verbose",     no_argument,       0, 'v'},
        {0, 0, 0, 0}
    };

    int option_index = 0;

    while (true)
    {
        int c = getopt_long(argc, argv, "l:c:s:i:d:e:g:hv", long_options, &option_index);

        if (c == -1)
            break;

        switch (c)
        {
            case 'l': populate_labels(optarg);
                      break;

            case 's': populate_selected_items(optarg);
                      break;

            case 'i': if(strlen(optarg) == 1)
                      {
                        live_input = atoi(optarg);
                      } else {
                        live_input = -1;
                        strcpy(video_clip, optarg);
                      }
                      break;

            case 'c': config_file = optarg;
                      break;

            case 'g': num_layers_groups = atoi(optarg);
                      assert(num_layers_groups >= 1 && num_layers_groups <= 2);
                      break;

            case 'd': num_dsps = atoi(optarg);
                      assert (num_dsps >= 0 && num_dsps <= 2);
                      break;

            case 'e': num_eves = atoi(optarg);
                      assert (num_eves >= 0 && num_eves <= 2);
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
    std::cout << "Usage: tidl_classification\n"
                 "  Will run all available networks if tidl is invoked without"
                 " any arguments.\n  Use -c to run a single network.\n"
                 "Optional arguments:\n"
                 " -c                   Path to the configuration file\n"
                 " -d <number of DSP cores> Number of DSP cores to use (0 - 2)\n"
                 " -e <number of EVE cores> Number of EVE cores to use (0 - 2)\n"
                 " -g <1|2>             Number of layer groups\n"
                 " -l                   List of label strings (of all classes in model)\n"
                 " -s                   List of strings with selected classes\n"
                 " -i                   Video input (for camera:0,1 or video clip)\n"
                 " -v                   Verbose output during execution\n"
                 " -h                   Help\n";
}

// Function to filter all the reported decisions
bool tf_expected_id(int id)
{
   // Filter out unexpected IDs
   for (int i = 0; i < selected_items_size; i ++)
   {
       if(id == selected_items[i]) return true;
   }
   return false;
}

int tf_postprocess(uchar *in, int size, int roi_idx, int frame_idx, int f_id)
{
  //prob_i = exp(TIDL_Lib_output_i) / sum(exp(TIDL_Lib_output))
  // sort and get k largest values and corresponding indices
  const int k = TOP_CANDIDATES;
  int rpt_id = -1;

  typedef std::pair<uchar, int> val_index;
  auto constexpr cmp = [](val_index &left, val_index &right) { return left.first > right.first; };
  std::priority_queue<val_index, std::vector<val_index>, decltype(cmp)> queue(cmp);
  // initialize priority queue with smallest value on top
  for (int i = 0; i < k; i++) {
    queue.push(val_index(in[i], i));
  }
  // for rest input, if larger than current minimum, pop mininum, push new val
  for (int i = k; i < size; i++)
  {
    if (in[i] > queue.top().first)
    {
      queue.pop();
      queue.push(val_index(in[i], i));
    }
  }

  // output top k values in reverse order: largest val first
  std::vector<val_index> sorted;
  while (! queue.empty())
   {
    sorted.push_back(queue.top());
    queue.pop();
  }

  for (int i = k-1; i >= 0; i--)
  {
      int id = sorted[i].second;

      if (tf_expected_id(id))
      {
        std::cout << "Frame:" << frame_idx << "," << f_id << " ROI[" << roi_idx << "]: rank="
                  << k-i << ", outval=" << (float)sorted[i].first / 255 << ", "
                  << labels_classes[sorted[i].second] << std::endl;
        rpt_id = id;
      }
  }
  return rpt_id;
}

void tf_preprocess(uchar *out, uchar *in, int size)
{
  for (int i = 0; i < size; i++)
  {
    out[i] = (uchar) (in[i] /*- 128*/);
  }
}

int ShowRegion(int roi_history[])
{
  if((roi_history[0] >= 0) && (roi_history[0] == roi_history[1])) return roi_history[0];
  if((roi_history[0] >= 0) && (roi_history[0] == roi_history[2])) return roi_history[0];
  if((roi_history[1] >= 0) && (roi_history[1] == roi_history[2])) return roi_history[1];
  return -1;
}


