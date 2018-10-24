/******************************************************************************
 * Copyright (c) 2018, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Texas Instruments Incorporated nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <getopt.h>
#include <cassert>
#include "video_utils.h"

using namespace std;
using namespace tidl;


bool ProcessArgs(int argc, char *argv[], cmdline_opts_t& opts)
{
    opts.num_frames      = 0;
    opts.output_width    = 0;
    opts.verbose         = false;
    opts.is_camera_input = false;
    opts.is_video_input  = false;

    const struct option long_options[] =
    {
        {"config",       required_argument, 0, 'c'},
        {"num_dsps",     required_argument, 0, 'd'},
        {"num_eves",     required_argument, 0, 'e'},
        {"num_layers_groups", required_argument, 0, 'g'},
        {"num_frames",   required_argument, 0, 'f'},
        {"input_file",   required_argument, 0, 'i'},
        {"output_width", required_argument, 0, 'w'},
        {"help",         no_argument,       0, 'h'},
        {"verbose",      no_argument,       0, 'v'},
        {0, 0, 0, 0}
    };

    int option_index = 0;

    while (true)
    {
        int c = getopt_long(argc, argv, "c:d:e:g:f:i:w:hv", long_options,
                            &option_index);

        if (c == -1)
            break;

        switch (c)
        {
            case 'c': opts.config = optarg;
                      break;

            case 'd': opts.num_dsps = atoi(optarg);
                      assert(opts.num_dsps >= 0 && opts.num_dsps <=
                                     Executor::GetNumDevices(DeviceType::DSP));
                      break;

            case 'e': opts.num_eves = atoi(optarg);
                      assert(opts.num_eves >= 0 && opts.num_eves <=
                                     Executor::GetNumDevices(DeviceType::EVE));
                      break;

            case 'g': opts.num_layers_groups = atoi(optarg);
                      assert((opts.num_layers_groups == 1) || (opts.num_layers_groups == 2) || (opts.num_layers_groups == 22));
                      break;

            case 'f': opts.num_frames = atoi(optarg);
                      assert (opts.num_frames > 0);
                      break;

            case 'i': opts.input_file = optarg;
                      break;

            case 'w': opts.output_width = atoi(optarg);
                      assert (opts.output_width > 0);
                      break;

            case 'v': opts.verbose = true;
                      break;

            case 'h': return false;
                      break;

            case '?': // Error in getopt_long
                      exit(EXIT_FAILURE);
                      break;

            default:
                      cerr << "Unsupported option: " << c << endl;
                      return false;
                      break;
        }
    }

    opts.is_camera_input = (opts.input_file.size() > 5 &&
                            opts.input_file.substr(0, 6) == "camera");
    if (opts.input_file.size() > 4)
    {
        string suffix = opts.input_file.substr(opts.input_file.size() - 4, 4);
        opts.is_video_input = (suffix == ".mp4") || (suffix == ".avi") ||
                              (suffix == ".mov");
    }

    return true;
}

// Set Video Input and Output
bool SetVideoInputOutput(VideoCapture &cap, const cmdline_opts_t& opts,
                         const char* window_name)
{
    if (opts.is_camera_input || opts.is_video_input)
    {
        if (opts.is_camera_input)
        {
            int port_num = 1;  // if TMDSCM572X camera module on AM57x EVM
            if (opts.input_file.size() > 6)  // "camera#"
                port_num = stoi(opts.input_file.substr(6,
                                                  opts.input_file.size() - 6));
            cap = VideoCapture(port_num);
        }
        else
            cap = VideoCapture(opts.input_file);
        if (! cap.isOpened())
        {
            cerr << "Cannot open video input: " << opts.input_file << endl;
            return false;
        }
        namedWindow(window_name, WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    }

    return true;
}

