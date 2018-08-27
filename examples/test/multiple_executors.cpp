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

//! @file multiple_executors.cpp
//! Illustrates how to setup multiple Executor instances using
//! non-overlapping sets of device ids and running the Executor instances
//! in parallel - each in its own thread

#include <signal.h>
#include <getopt.h>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <functional>
#include <algorithm>
#include <pthread.h>

#include "executor.h"
#include "execution_object.h"
#include "configuration.h"
#include "utils.h"

using namespace tidl;

extern
bool RunNetwork(DeviceType           device_type,
                const DeviceIds&     ids,
                const Configuration& c,
                std::istream&        input,
                std::ostream&        output);

void* run_network(void *data);

struct ThreadArg
{
    ThreadArg(const DeviceIds& device_ids, const std::string& s):
        ids(device_ids), config_file(s) {}

    DeviceIds   ids;
    std::string config_file;
};

bool thread_status[2];

bool RunMultipleExecutors(const std::string& config_file_1,
                          const std::string& config_file_2,
                          uint32_t num_devices_available)
{
    // If there is only 1 device available, skip
    if (num_devices_available == 1)
        return true;

    DeviceIds ids1, ids2;

    if (num_devices_available == 4)
    {
        ids1 = {DeviceId::ID2, DeviceId::ID3};
        ids2 = {DeviceId::ID0, DeviceId::ID1};
    }
    else
    {
        ids1 = {DeviceId::ID0};
        ids2 = {DeviceId::ID1};
    }

    // Set up devices and config files for each thread
    ThreadArg arg1(ids2, config_file_1);
    ThreadArg arg2(ids1, config_file_2);

    // Run network 1 in a thread
    std::cout << std::endl << "Multiple Executor..." << std::endl;
    std::cout << "Running network "
              << arg1.config_file.substr(arg1.config_file.find("tidl"))
              << " on EVEs: ";
    for (DeviceId id : arg1.ids)
        std::cout << static_cast<int>(id) << " ";
    std::cout << " in thread 0" << std::endl;

    pthread_t network_thread_1;
    pthread_create(&network_thread_1, 0, &run_network, &arg1);

    // Run network 2 in a thread
    std::cout << "Running network "
              << arg2.config_file.substr(arg2.config_file.find("tidl"))
              << " on EVEs: ";
    for (DeviceId id : arg2.ids)
        std::cout << static_cast<int>(id) << " ";
    std::cout << " in thread 1" << std::endl;

    pthread_t network_thread_2;
    pthread_create(&network_thread_2, 0, &run_network, &arg2);

    // Wait for both networks to complete
    void *thread_return_val1;
    void *thread_return_val2;
    pthread_join(network_thread_1, &thread_return_val1);
    pthread_join(network_thread_2, &thread_return_val2);

    if (thread_return_val1 == 0 || thread_return_val2 == 0)
    {
        std::cout << "Multiple executors: FAILED" << std::endl;
        return false;
    }

    std::cout << "Multiple executors: PASSED" << std::endl;
    return true;
}


void* run_network(void *data)
{
    const ThreadArg* arg = static_cast<const ThreadArg *>(data);

    const DeviceIds& ids = arg->ids;
    const std::string& config_file = arg->config_file;

    // Read the TI DL configuration file
    Configuration configuration;
    bool status = configuration.ReadFromFile(config_file);
    assert (status != false);

    configuration.outData += std::to_string(pthread_self());

    // Open input and output files
    std::ifstream input_data_file(configuration.inData, std::ios::binary);
    std::ofstream output_data_file(configuration.outData, std::ios::binary);
    assert (input_data_file.good());
    assert (output_data_file.good());

    RunNetwork(DeviceType::EVE, ids, configuration,
               input_data_file, output_data_file);

    input_data_file.close();
    output_data_file.close();

    // Return 1 for true, 0 for false. void * pattern follows example from:
    // "Advanced programming in the Unix Environment"
    if (!status) return ((void *)0);

    return ((void *)1);
}
