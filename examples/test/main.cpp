/******************************************************************************
 * Copyright (c) 2017, Texas Instruments Incorporated - http://www.ti.com/
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

#include "executor.h"
#include "execution_object.h"
#include "configuration.h"

bool __TI_show_debug_ = false;

using namespace tidl;

bool RunMultipleExecutors(const std::string& config_file_1,
                          const std::string& config_file_2,
                          uint32_t num_devices_available);

bool RunConfiguration(const std::string& config_file, int num_devices,
                      DeviceType device_type);
bool RunAllConfigurations(int32_t num_devices, DeviceType device_type);

bool ReadFrame(ExecutionObject&     eo,
               int                  frame_idx,
               const Configuration& configuration,
               std::istream&        input_file);

bool WriteFrame(const ExecutionObject &eo,
                std::ostream& output_file);

static void ProcessArgs(int argc, char *argv[],
                        std::string& config_file,
                        int& num_devices,
                        DeviceType& device_type);

static void DisplayHelp();

static double ms_diff(struct timespec &t0, struct timespec &t1)
{ return (t1.tv_sec - t0.tv_sec) * 1e3 + (t1.tv_nsec - t0.tv_nsec) / 1e6; }


int main(int argc, char *argv[])
{
    // Catch ctrl-c to ensure a clean exit
    signal(SIGABRT, exit);
    signal(SIGTERM, exit);

    // If there are no devices capable of offloading TIDL on the SoC, exit
    uint32_t num_dla =
                Executor::GetNumDevicesSupportingTIDL(DeviceType::DLA);
    uint32_t num_dsp =
                Executor::GetNumDevicesSupportingTIDL(DeviceType::DSP);
    if (num_dla == 0 && num_dsp == 0)
    {
        std::cout << "TI DL not supported on this SoC." << std::endl;
        return EXIT_SUCCESS;
    }

    // Process arguments
    std::string config_file;
    int         num_devices = 1;
    DeviceType  device_type = DeviceType::DLA;
    ProcessArgs(argc, argv, config_file, num_devices, device_type);

    bool status = true;
    if (!config_file.empty())
        status = RunConfiguration(config_file, num_devices, device_type);
    else
    {
        if (num_dla > 0)
        {
            //TODO: Use memory availability to determine # devices
            // Run on 2 devices because there is not enough CMEM available by
            // default
            if (num_dla = 4) num_dla = 2;
            status = RunAllConfigurations(num_dla, DeviceType::DLA);
            status &= RunMultipleExecutors(
                     "testvecs/config/infer/tidl_config_j11_v2.txt",
                     "testvecs/config/infer/tidl_config_j11_cifar.txt",
                     num_dla);
        }

        if (num_dsp > 0)
        {
            status &= RunAllConfigurations(num_dsp, DeviceType::DSP);
        }
    }

    if (!status)
    {
        std::cout << "tidl FAILED" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "tidl PASSED" << std::endl;
    return EXIT_SUCCESS;
}

bool RunConfiguration(const std::string& config_file, int num_devices,
                      DeviceType device_type)
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

    // Open input and output files
    std::ifstream input_data_file(configuration.inData, std::ios::binary);
    std::ofstream output_data_file(configuration.outData, std::ios::binary);
    assert (input_data_file.good());
    assert (output_data_file.good());

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
            ArgInfo in  = { ArgInfo(malloc_ddr<char>(frame_sz), frame_sz)};
            ArgInfo out = { ArgInfo(malloc_ddr<char>(frame_sz), frame_sz)};
            eo->SetInputOutputBuffer(in, out);

            buffers.push_back(in.ptr());
            buffers.push_back(out.ptr());
        }

        #define MAX_NUM_EOS  4
        struct timespec t0[MAX_NUM_EOS], t1;

        // Process frames with available execution objects in a pipelined manner
        // additional num_eos iterations to flush the pipeline (epilogue)
        for (int frame_idx = 0;
             frame_idx < configuration.numFrames + num_eos; frame_idx++)
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

                WriteFrame(*eo, output_data_file);
            }

            // Read a frame and start processing it with current eo
            if (ReadFrame(*eo, frame_idx, configuration, input_data_file))
            {
                clock_gettime(CLOCK_MONOTONIC, &t0[frame_idx % num_eos]);
                eo->ProcessFrameStartAsync();
            }
        }

        for (auto b : buffers)
            __free_ddr(b);

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

namespace tidl {
extern bool CompareFiles (const std::string &F1, const std::string &F2);
extern bool CompareFrames(const std::string &F1, const std::string &F2,
                          int numFrames, int width, int height);
}

bool RunAllConfigurations(int32_t num_devices, DeviceType device_type)
{
    std::vector<std::string> configurations;

    if (device_type == DeviceType::DLA)
        configurations = {"dense_1x1",  "j11_bn", "j11_cifar",
                          "j11_controlLayers", "j11_prelu", "j11_v2",
                          "jseg21", "jseg21_tiscapes", "smallRoi", "squeeze1_1"};
    else
        configurations = {"j11_bn",
                          "j11_controlLayers", "j11_v2",
                          "jseg21", "jseg21_tiscapes", "smallRoi", "squeeze1_1"};

    int errors = 0;
    for (auto config : configurations)
    {
        std::string config_file = "testvecs/config/infer/tidl_config_"
                                  + config + ".txt";
        std::cout << "Running " << config << " on " << num_devices
                  << " devices, type "
                  << ((device_type == DeviceType::DLA) ? "EVE" : "DSP")
                  << std::endl;

        Configuration configuration;
        bool status = configuration.ReadFromFile(config_file);
        if (!status) { errors++; continue; }

        status = RunConfiguration(config_file, num_devices, device_type);

        if (!status) { errors++; continue; }

        // Check output against reference output
        std::string reference_output = "testvecs/reference/"
                                       + config + "_ref.bin";

        // Reference for jseg21_tiscapes only has one frame
        if (config.compare("jseg21_tiscapes") == 0)
            status = CompareFrames(configuration.outData, reference_output,
                                   1, 1024, 512);
        else
            status = CompareFiles(configuration.outData, reference_output);

        if (status) std::cout << config << " : PASSED" << std::endl;
        else        std::cout << config << " : FAILED" << std::endl;

        if (!status) errors++;
    }

    if (errors > 0) return false;

    return true;
}



bool ReadFrame(ExecutionObject &eo, int frame_idx,
               const Configuration& configuration,
               std::istream& input_file)
{
    if (frame_idx >= configuration.numFrames)
        return false;

    char*  frame_buffer = eo.GetInputBufferPtr();
    assert (frame_buffer != nullptr);

    input_file.read(eo.GetInputBufferPtr(),
                    eo.GetInputBufferSizeInBytes());

    if (input_file.eof())
        return false;

    assert (input_file.good());

    // Set the frame index  being processed by the EO. This is used to
    // sort the frames before they are output
    eo.SetFrameIndex(frame_idx);

    if (input_file.good())
        return true;

    return false;
}

bool WriteFrame(const ExecutionObject &eo, std::ostream& output_file)
{
    output_file.write(
            eo.GetOutputBufferPtr(), eo.GetOutputBufferSizeInBytes());
    assert(output_file.good() == true);

    if (output_file.good())
        return true;

    return false;
}


void ProcessArgs(int argc, char *argv[], std::string& config_file,
                 int& num_devices, DeviceType& device_type)
{
    const struct option long_options[] =
    {
        {"config_file", required_argument, 0, 'c'},
        {"num_devices", required_argument, 0, 'n'},
        {"device_type", required_argument, 0, 't'},
        {"help",        no_argument,       0, 'h'},
        {"verbose",     no_argument,       0, 'v'},
        {0, 0, 0, 0}
    };

    int option_index = 0;

    while (true)
    {
        int c = getopt_long(argc, argv, "c:n:t:hv", long_options, &option_index);

        if (c == -1)
            break;

        switch (c)
        {
            case 'c': config_file = optarg;
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
    std::cout << "Usage: tidl\n"
                 "  Will run all available networks if tidl is invoked without"
                 " any arguments.\n  Use -c to run a single network.\n"
                 "Optional arguments:\n"
                 " -c                   Path to the configuration file\n"
                 " -n <number of cores> Number of cores to use (1 - 4)\n"
                 " -t <d|e>             Type of core. d -> DSP, e -> DLA\n"
                 " -v                   Verbose output during execution\n"
                 " -h                   Help\n";
}
