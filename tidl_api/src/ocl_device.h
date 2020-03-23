/******************************************************************************
 * Copyright (c) 2017-2018, Texas Instruments Incorporated - http://www.ti.com/
 *   All rights reserved.
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

/*! \file ocl_device.h
 *  \brief Wrapper classes for OpenCL C structures
 *
 *  Provides a high level abstraction for OpenCL APIs
 */

#pragma once
#include <CL/TI/cl.h>
#include <CL/TI/cl_ext.h>
#include <vector>
#include <memory>
#include "executor.h"
#include "device_arginfo.h"
#include "parameters.h"

namespace tidl
{

typedef std::vector<DeviceArgInfo> KernelArgs;

class Kernel;

/*! \brief Manages OpenCL context, device and command queues
 *
 *  Inititalizes an OpenCL context, creates devices and command queues to
 *  each device.
 */
class Device
{

    public:
        typedef std::unique_ptr<Device> Ptr;

        Device(cl_device_type t, const DeviceIds& ids);
        virtual ~Device();


        static Ptr Create(DeviceType core_type, const DeviceIds& ids,
                          const std::string& name);

        cl_command_queue& GetCommandQueue(uint8_t index)
                            { return queue_m[index]; }

        cl_device_type type() const { return device_type_m; }

        float GetFrequencyInMhz() const { return freq_in_mhz_m; }

        static uint32_t GetNumDevices(DeviceType device_type);

        virtual std::string GetDeviceName() = 0;

    protected:

        static const int MAX_DEVICES = 4;
        cl_mem CreateBuffer(const DeviceArgInfo &Arg);
        void   ReleaseBuffer(cl_mem M);


              cl_context        context_m;
              cl_program        program_m;
              cl_command_queue  queue_m[MAX_DEVICES];
        const cl_device_type    device_type_m;
        const DeviceIds         device_ids_m;
              cl_uint           freq_in_mhz_m;

        friend Kernel;
};

class DspDevice: public Device
{
    public:
        DspDevice(const DeviceIds& ids, const std::string &binary_filename);
        virtual ~DspDevice() {}

        DspDevice()                            = delete;
        DspDevice(const DspDevice&)            = delete;
        DspDevice& operator=(const DspDevice&) = delete;

        virtual std::string GetDeviceName() { return "DSP"; }

    protected:
        bool BuildProgramFromBinary(const std::string &binary_filename,
                                    cl_device_id device_ids[],
                                    int num_devices);
};

class EveDevice : public Device
{
    public:
        EveDevice(const DeviceIds& ids, const std::string &kernel_names);
        virtual ~EveDevice() {}

        EveDevice()                            = delete;
        EveDevice(const EveDevice&)            = delete;
        EveDevice& operator=(const EveDevice&) = delete;

        virtual std::string GetDeviceName() { return "EVE"; }

    protected:
        bool BuildProgramFromBinary(const std::string &kernel_names,
                                    cl_device_id device_ids[],
                                    int num_devices);

};


/*! \brief OpenCL kernels
 *
 * Create and execute OpenCL kernels
 */
class Kernel
{
    public:
        Kernel(Device *device, const std::string &Name,
               const KernelArgs &args, uint8_t device_index);
        ~Kernel();

        bool UpdateScalarArg(uint32_t index, size_t size, const void *value);
        Kernel& RunAsync(uint32_t context_idx = 0);
        bool Wait(uint32_t context_idx = 0);
        bool AddCallback(void *user_data, uint32_t context_idx = 0);

    private:
        cl_kernel           kernel_m;
        cl_event            event_m[tidl::internal::NUM_CONTEXTS];
        std::vector<cl_mem> buffers_m;
        const std::string   name_m;

        Device*             device_m;
        uint8_t             device_index_m;
};


} // namespace oa
