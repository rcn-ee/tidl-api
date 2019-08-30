/******************************************************************************
 * Copyright (c) 2017-2018  Texas Instruments Incorporated - http://www.ti.com/
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
#include <fstream>
#include <cassert>
#include <string>
#include <vector>

#include "executor.h"
#include "execution_object.h"
#include "configuration.h"
#include "utils.h"

using namespace tidl;
using std::string;

bool RunMultipleExecutors(const string& config_file_1,
                          const string& config_file_2,
                          uint32_t      num_devices_available);

bool RunConfiguration(const string& config_file,
                      int           num_devices,
                      DeviceType    device_type);

bool RunAllConfigurations(int32_t    num_devices,
                          DeviceType device_type);

bool RunNetwork(DeviceType           device_type,
                const DeviceIds&     ids,
                const Configuration& c,
                std::istream&        input,
                std::ostream&        output);

static void ProcessArgs(int argc, char *argv[],
                        string&     config_file,
                        int&        num_devices,
                        DeviceType& device_type);

static void DisplayHelp();

bool verbose = false;

int main(int argc, char *argv[])
{
    // Catch ctrl-c to ensure a clean exit
    signal(SIGABRT, exit);
    signal(SIGTERM, exit);

    // If there are no devices capable of offloading TIDL on the SoC, exit
    uint32_t num_eve = Executor::GetNumDevices(DeviceType::EVE);
    uint32_t num_dsp = Executor::GetNumDevices(DeviceType::DSP);
    if (num_eve == 0 && num_dsp == 0)
    {
        std::cout << "TI DL not supported on this processor." << std::endl;
        return EXIT_SUCCESS;
    }

    // Process arguments
    string      config_file;
    int         num_devices = 1;
    DeviceType  device_type = DeviceType::EVE;
    ProcessArgs(argc, argv, config_file, num_devices, device_type);

    bool status = true;
    if (!config_file.empty())
        status = RunConfiguration(config_file, num_devices, device_type);
    else
    {
        if (num_eve > 0)
        {
            // Run on 2 devices because there is not enough CMEM available by
            // default
            if (num_eve == 4)
            {
                std::cout
                 << "Running on 2 EVE devices instead of the available 4 "
                 << "due to insufficient OpenCL global memory. Refer the "
                 << "TIDL API User's Guide, Frequently Asked Questions, "
                 << "Section \"Insufficient OpenCL global memory\" for details "
                 << "on increasing the amount of CMEM available for OpenCL."
                 << std::endl;
                num_eve = 2;
            }
            status = RunAllConfigurations(num_eve, DeviceType::EVE);
            status &= RunMultipleExecutors(
                     "testvecs/config/infer/tidl_config_j11_v2.txt",
                     "testvecs/config/infer/tidl_config_j11_cifar.txt",
                     num_eve);
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
    Configuration c;
    if (!c.ReadFromFile(config_file)) return false;
    if (verbose)                      c.enableApiTrace = true;

    // Open input and output files
    std::ifstream input_data_file(c.inData, std::ios::binary);
    std::ofstream output_data_file(c.outData, std::ios::binary);
    assert (input_data_file.good());
    assert (output_data_file.good());

    bool status = RunNetwork(device_type, ids, c,
                             input_data_file, output_data_file);

    input_data_file.close();
    output_data_file.close();

    return status;
}

bool RunNetwork(DeviceType           device_type,
                const DeviceIds&     ids,
                const Configuration& c,
                std::istream&        input,
                std::ostream&        output)
{
    bool status = true;

    try
    {
        // Create a executor with the specified core type, number of cores
        // and configuration
        Executor E(device_type, ids, c);

        std::vector<ExecutionObject *> EOs;
        for (unsigned int i = 0; i < E.GetNumExecutionObjects(); i++)
            EOs.push_back(E[i]);

        int num_eos = EOs.size();

        // Allocate input and output buffers for each execution object
        for (auto eo : EOs)
        {
            size_t in_size  = eo->GetInputBufferSizeInBytes();
            size_t out_size = eo->GetOutputBufferSizeInBytes();
            ArgInfo in  = { ArgInfo(malloc(in_size),  in_size)};
            ArgInfo out = { ArgInfo(malloc(out_size), out_size)};
            eo->SetInputOutputBuffer(in, out);
        }

        // Process frames with available execution objects in a pipelined manner
        // additional num_eos iterations to flush the pipeline (epilogue)
        for (int frame_idx = 0; frame_idx < c.numFrames + num_eos; frame_idx++)
        {
            ExecutionObject* eo = EOs[frame_idx % num_eos];

            // Wait for previous frame on the same eo to finish processing
            if (eo->ProcessFrameWait())
            {
                ReportTime(eo);
                WriteFrame(eo, output);
            }

            // Read a frame and start processing it with current eo
            if (ReadFrame(eo, frame_idx, c, input))
                eo->ProcessFrameStartAsync();
        }

        for (auto eo : EOs)
        {
            free(eo->GetInputBufferPtr());
            free(eo->GetOutputBufferPtr());
        }
    }
    catch (tidl::Exception &e)
    {
        std::cerr << e.what() << std::endl;
        status = false;
    }

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

    if (device_type == DeviceType::EVE)
        configurations = {"dense_1x1",  "j11_bn", "j11_cifar",
                          "j11_controlLayers", "j11_prelu", "j11_v2",
                          "jseg21", "jseg21_tiscapes", "smallRoi", "squeeze1_1"};
    else
        configurations = {"dense_1x1",  "j11_bn", "j11_cifar",
                          "j11_controlLayers", "j11_v2",
                          "jseg21", "jseg21_tiscapes", "smallRoi", "squeeze1_1"};

    int errors = 0;
    for (auto config : configurations)
    {
        // Skip smallRoi, not a real network anyway
        if (config.compare("smallRoi") == 0)  continue;

        std::string config_file = "testvecs/config/infer/tidl_config_"
                                  + config + ".txt";
        std::cout << "Running " << config << " on " << num_devices
                  << " devices, type "
                  << ((device_type == DeviceType::EVE) ? "EVE" : "DSP")
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
                          device_type = DeviceType::EVE;
                      else if (*optarg == 'd')
                          device_type = DeviceType::DSP;
                      else
                      {
                          std::cerr << "Invalid argument to -t, only e or d"
                                       " allowed" << std::endl;
                          exit(EXIT_FAILURE);
                      }
                      break;

            case 'v': verbose = true;
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
    std::cout << "Usage: test_tidl\n"
                 "  Will run all available networks if invoked without"
                 " any arguments.\n  Use -c to run a single network.\n"
                 "Optional arguments:\n"
                 " -c                   Path to the configuration file\n"
                 " -n <number of cores> Number of cores to use (1 - 4)\n"
                 " -t <d|e>             Type of core. d -> DSP, e -> EVE\n"
                 " -v                   Verbose output during execution\n"
                 " -h                   Help\n";
}
