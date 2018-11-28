/******************************************************************************
 * Copyright (c) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of Texas Instruments Incorporated nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *  THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once

#include <assert.h>
#include <type_traits>
#include <cstddef>
#include <vector>
#include <memory>
#include <fstream>

#include "configuration.h"
#include "ocl_device.h"

#include "common_defines.h"
#include "tidl_create_params.h" // for TIDL types
#include "execution_object.h"

namespace tidl {

typedef std::vector<std::unique_ptr<ExecutionObject>> ExecutionObjects;

// One instance across all devices available in the context
// Also need this to work in host emulation mode
class ExecutorImpl
{
    public:
        ExecutorImpl(DeviceType core_type, const DeviceIds& ids,
                     int layersGroupId);
        ~ExecutorImpl() { Cleanup(); }

        bool Initialize(const Configuration& configuration);

        ExecutorImpl(const ExecutorImpl&)            = delete;
        ExecutorImpl& operator=(const ExecutorImpl&) = delete;

        ExecutionObjects execution_objects_m;

    private:
        void InitializeNetworkCreateParam(TIDL_CreateParams *cp,
                                          const Configuration& c);
        bool InitializeNetworkParams(TIDL_CreateParams *cp);
        void Cleanup();

        Device::Ptr          device_m; // vector of devices?
        Configuration        configuration_m;
        up_malloc_ddr<char>  shared_networkparam_heap_m;
        DeviceIds            device_ids_m;
        DeviceType           core_type_m;
        int                  layers_group_id_m;
};

} // namespace tidl
