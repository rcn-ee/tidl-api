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

//! @file device_arginfo.h

#pragma once

#include "executor.h"
#include <memory>

namespace tidl
{

/*! @class DeviceArgInfo
 *  @brief Describe input and output buffers required by ExecutionObjects
 */
class DeviceArgInfo: public ArgInfo
{
    public:
        //! Enumerates the types of arguments represented by DeviceArgInfo
        enum class Kind { BUFFER=0, LOCAL, SCALAR };

        //! Construct an DeviceArgInfo object
        DeviceArgInfo(const ArgInfo& ai, Kind kind) :
            ArgInfo(ai), kind_m(kind)
        {}

        DeviceArgInfo(void *p, size_t size, Kind kind) :
            ArgInfo(p, size), kind_m(kind)
        {}

        Kind   kind() const { return kind_m; }
        bool   isLocal() const { return (ptr_m == nullptr) && (size_m > 0); }

    private:
        Kind         kind_m;
};

/*! @class PipeInfo
 *  @brief Describe input and output required by piping output and input
 *         between Execution Objects
 */
class PipeInfo
{
    public:
        uint32_t dataQ_m[OCL_TIDL_MAX_IN_BUFS];
};

/*! @class IODeviceArgInfo
 *  @brief Describe input and output buffers by an Execution Object (EO)
 *         Also used to chain execution objects - the output buffer of a
 *         producer EO is the same as the input buffer of a consumer EO.
 *         The PipeInfo must be shared across the producer and consumer EO,
 *         hence the shared pointer.
 */
class IODeviceArgInfo
{
    public:
        explicit IODeviceArgInfo(const ArgInfo& arg):
                        arg_m(arg, DeviceArgInfo::Kind::BUFFER)
        {
            pipe_m = std::make_shared<PipeInfo>();
        }

        IODeviceArgInfo(): arg_m(nullptr, 0, DeviceArgInfo::Kind::BUFFER)
        {
            pipe_m = nullptr;
        }

        PipeInfo&            GetPipe()      { return *pipe_m; }
        const DeviceArgInfo& GetArg() const { return arg_m; }

        //IODeviceArgInfo(const IODeviceArgInfo&)            = delete;
        //IODeviceArgInfo& operator=(const IODeviceArgInfo&) = delete;

    private:
        DeviceArgInfo             arg_m;
        std::shared_ptr<PipeInfo> pipe_m;
};



} //namespace
