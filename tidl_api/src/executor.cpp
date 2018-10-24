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

#include <assert.h>
#include "executor.h"
#include "executor_impl.h"
#include "parameters.h"
#include "util.h"
#include "trace.h"


using namespace tidl;

using std::unique_ptr;

Executor::Executor(DeviceType core_type, const DeviceIds& ids,
                   const Configuration& configuration, int layers_group_id)
{
    TRACE::enabled = configuration.enableApiTrace;

    TRACE::print("-> Executor::Executor()\n");

    pimpl_m = unique_ptr<ExecutorImpl>
              { new ExecutorImpl(core_type, ids, layers_group_id) };
    pimpl_m->Initialize(configuration);

    TRACE::print("<- Executor::Executor()\n");
}



// Pointer to implementation idiom: https://herbsutter.com/gotw/_100/:
// Both unique_ptr and shared_ptr can be instantiated with an incomplete type
// unique_ptr's destructor requires a complete type in order to invoke delete
// By writing it yourself in the implementation file, you force it to be
// defined in a place where impl is already defined, and this successfully
// prevents the compiler from trying to automatically generate the destructor
// on demand in the caller’s code where impl is not defined.
Executor::~Executor() = default;

uint32_t Executor::GetNumDevices(DeviceType device_type)
{
    return Device::GetNumDevices(device_type);
}

#define STRING(S)  XSTRING(S)
#define XSTRING(S) #S
std::string Executor::GetAPIVersion()
{
    static std::string version = STRING(_BUILD_VER);
    version += ".";
    version += STRING(_BUILD_SHA);
    return version;
}


ExecutorImpl::ExecutorImpl(DeviceType core_type, const DeviceIds& ids,
                           int layers_group_id):
    configuration_m(),
    shared_networkparam_heap_m(nullptr, &__free_ddr),
    device_ids_m(ids),
    core_type_m(core_type),
    layers_group_id_m(layers_group_id)
{
    std::string name;
    if (core_type_m == DeviceType::DSP)
        name  = "";
    else if (core_type_m == DeviceType::EVE)
        name = STRING(SETUP_KERNEL) ";" STRING(INIT_KERNEL) ";" STRING(PROCESS_KERNEL) ";" STRING(CLEANUP_KERNEL);

    device_m = Device::Create(core_type_m, ids, name);
}

ExecutionObject* Executor::operator[](uint32_t index) const
{
    assert(index < pimpl_m->execution_objects_m.size());
    return pimpl_m->execution_objects_m[index].get();
}

uint32_t Executor::GetNumExecutionObjects() const
{
    return pimpl_m->execution_objects_m.size();
}

bool ExecutorImpl::Initialize(const Configuration& configuration)
{
    configuration_m = configuration;

    // Allocate, initialize TIDL_CreateParams object
    up_malloc_ddr<TIDL_CreateParams> shared_createparam(
                                            malloc_ddr<TIDL_CreateParams>(),
                                            &__free_ddr);
    InitializeNetworkCreateParam(shared_createparam.get(), configuration);

    // Read network from file into network struct in TIDL_CreateParams
    sTIDL_Network_t *net = &(shared_createparam.get())->net;

    bool status = ReadBinary(configuration_m.netBinFile,
                             reinterpret_cast<char *>(net),
                             sizeof(sTIDL_Network_t));
    assert(status != false);

    // Force to run full network if runFullNet is set
    if (configuration.runFullNet)
    {
        for (int i = 0; i < net->numLayers; i++)
            if (net->TIDLLayers[i].layerType != TIDL_DataLayer)
                net->TIDLLayers[i].layersGroupId = layers_group_id_m;
    }

    // If the user has specified an override mapping, apply it
    else if (!configuration.layerIndex2LayerGroupId.empty())
    {
        for (const auto &item : configuration.layerIndex2LayerGroupId)
            if (item.first < net->numLayers)
                net->TIDLLayers[item.first].layersGroupId = item.second;
    }

    // Call a setup kernel to allocate and fill network parameters
    InitializeNetworkParams(shared_createparam.get());

    const ArgInfo create_arg(shared_createparam.get(),
                             sizeof(TIDL_CreateParams));
    const ArgInfo param_heap_arg(shared_networkparam_heap_m.get(),
                                 configuration_m.PARAM_HEAP_SIZE);
    for (auto ids : device_ids_m)
    {
        uint8_t index = static_cast<uint8_t>(ids);
        execution_objects_m.push_back(
             unique_ptr<ExecutionObject>
             {new ExecutionObject(device_m.get(), core_type_m, index,
                                  create_arg, param_heap_arg,
                                  configuration_m,
                                  layers_group_id_m)} );
    }

    for (auto &eo : execution_objects_m)
        eo->RunAsync(ExecutionObject::CallType::INIT);

    for (auto &eo : execution_objects_m)
        eo->Wait(ExecutionObject::CallType::INIT);

    return true;
}


bool ExecutorImpl::InitializeNetworkParams(TIDL_CreateParams *cp)
{
    // Determine size of network parameters buffer, allocate it
    size_t networkparam_size =
                        GetBinaryFileSize(configuration_m.paramsBinFile);

    up_malloc_ddr<char> networkparam(malloc_ddr<char>(networkparam_size),
                                &__free_ddr);

    // Read network parameters from bin file into buffer
    bool status = ReadBinary(configuration_m.paramsBinFile,
                             networkparam.get(),
                             networkparam_size);
    assert(status != false);

    // Allocate a buffer for passing parameters to the kernel
    up_malloc_ddr<OCL_TIDL_SetupParams> setupParams(
                                            malloc_ddr<OCL_TIDL_SetupParams>(),
                                            &__free_ddr);

    // Set up execution trace specified in the configuration
    EnableExecutionTrace(configuration_m, &setupParams->enableTrace);

    setupParams->networkParamHeapSize = configuration_m.PARAM_HEAP_SIZE;
    setupParams->noZeroCoeffsPercentage = configuration_m.noZeroCoeffsPercentage;
    setupParams->sizeofTIDL_CreateParams = sizeof(TIDL_CreateParams);
    setupParams->offsetofNet = offsetof(TIDL_CreateParams, net);

    // Allocate buffer for a network parameter heap. Used by the setup
    // kernel to allocate and initialize network parameters for the layers
    shared_networkparam_heap_m.reset(malloc_ddr<char>(setupParams->networkParamHeapSize));

    KernelArgs args = { DeviceArgInfo(cp, sizeof(TIDL_CreateParams),
                                      DeviceArgInfo::Kind::BUFFER),
                        DeviceArgInfo(networkparam.get(), networkparam_size,
                                      DeviceArgInfo::Kind::BUFFER),
                        DeviceArgInfo(shared_networkparam_heap_m.get(),
                                      setupParams->networkParamHeapSize,
                                      DeviceArgInfo::Kind::BUFFER),
                        DeviceArgInfo(setupParams.get(),
                                      sizeof(OCL_TIDL_SetupParams),
                                      DeviceArgInfo::Kind::BUFFER) };

    // Execute kernel on first available device in the Executor
    uint8_t id = static_cast<uint8_t>(*(device_ids_m.cbegin()));
    unique_ptr<Kernel> K {new Kernel(device_m.get(), STRING(SETUP_KERNEL),
                                     args, id)};
    K->RunAsync();
    K->Wait();

    if (setupParams->errorCode != OCL_TIDL_SUCCESS)
        throw Exception(setupParams->errorCode,
                        __FILE__, __FUNCTION__, __LINE__);

    return status;
}


void ExecutorImpl::Cleanup()
{
    for (auto &eo : execution_objects_m)
        eo->RunAsync(ExecutionObject::CallType::CLEANUP);

    for (auto &eo : execution_objects_m)
        eo->Wait(ExecutionObject::CallType::CLEANUP);
}


void ExecutorImpl::InitializeNetworkCreateParam(TIDL_CreateParams *CP,
                                                const Configuration& c)
{
    CP->currCoreId           = layers_group_id_m;
    CP->currLayersGroupId    = layers_group_id_m;
    CP->l1MemSize            = tidl::internal::DMEM0_SIZE;
    CP->l2MemSize            = tidl::internal::DMEM1_SIZE;
    CP->l3MemSize            = tidl::internal::OCMC_SIZE;

    CP->quantHistoryParam1   = c.quantHistoryParam1;
    CP->quantHistoryParam2   = c.quantHistoryParam2;
    CP->quantMargin          = c.quantMargin;

    // If trace is enabled, setup the device TIDL library to allocate separate
    // output buffers for each layer. This makes it possible for the host
    // to access the output of each layer after a frame is processed.
    if (configuration_m.enableOutputTrace)
        CP->optimiseExtMem       = TIDL_optimiseExtMemL0;
    else
        CP->optimiseExtMem       = TIDL_optimiseExtMemL1;
}

Exception::Exception(const std::string& error, const std::string& file,
                     const std::string& func, uint32_t line_no)
{

    message_m = "TIDL Error: [";
    message_m += file;
    message_m += ", ";
    message_m += func;
    message_m += ", ";
    message_m += std::to_string(line_no);
    message_m += "]: ";
    message_m += error;
}

// Refer ti-opencl/builtins/include/custom.h for error codes
Exception::Exception(int32_t errorCode, const std::string& file,
                     const std::string& func, uint32_t line_no)
{
    message_m = "TIDL Error: [";
    message_m += file;
    message_m += ", ";
    message_m += func;
    message_m += ", ";
    message_m += std::to_string(line_no);
    message_m += "]: ";

    switch (errorCode)
    {
        case OCL_TIDL_ERROR:
        message_m += "";
            break;
        case OCL_TIDL_ALLOC_FAIL:
        case OCL_TIDL_MEMREC_ALLOC_FAIL:
            message_m += "Memory allocation failed on device";
            break;
        case OCL_TIDL_PROCESS_FAIL:
        message_m += "Process call failed on device";
            break;
        case OCL_TIDL_CREATE_PARAMS_MISMATCH:
            message_m += "TIDL API headers inconsistent with OpenCL";
            break;
        case OCL_TIDL_INIT_FAIL:
            message_m += "Initialization failed on device";
            break;
        default:
        message_m += std::to_string(errorCode);
            break;
    }
}

const char* Exception::what() const noexcept
{
    return message_m.c_str();
}
