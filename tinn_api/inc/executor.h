/******************************************************************************
 * Copyright (c) 2017-18 Texas Instruments Incorporated - http://www.ti.com/
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

//! @file executor.h

#pragma once
#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <cassert>
#include <set>
#include <exception>

#include "configuration.h"

namespace tidl {

//! Enumerates types of devices available to offload the network.
enum class DeviceType { DSP, /**< Offload to C66x DSP */
                        DLA  /**< Offload to TI DLA */
                      };

//! Enumerates IDs for devices of a given type.
enum class DeviceId : int { ID0=0, /**< DSP1 or DLA1 */
                            ID1,   /**< DSP2 or DLA2 */
                            ID2,   /**< DLA3 */
                            ID3    /**< DLA4 */
                          };

//! Used to specify the set of devices available to an Executor
typedef std::set<DeviceId> DeviceIds;

class ExecutorImpl;
class ExecutionObject;

//! Defines the return type for Executor::GetExecutionObjects
typedef std::vector<std::unique_ptr<ExecutionObject>> ExecutionObjects;

/*! @class Executor
    @brief Manages the overall execution of a network using the
    specified configuration and the set of devices available to the
    executor.
*/
class Executor
{
    public:
        //! @brief Create an Executor object.
        //!
        //! The Executor will create the required ExecutionObject's and
        //! initialize them with the specified TI DL network. E.g.
        //! @code
        //!   Configuration configuration;
        //!   configuration.ReadFromFile("path to configuration file");
        //!   DeviceIds ids1 = {DeviceId::ID2, DeviceId::ID3};
        //!   Executor executor(DeviceType::DLA, ids, configuration);
        //! @endcode
        //!
        //! @param device_type DSP or EVE/DLA device
        //! @param ids Set of devices uses by this instance of the Executor
        //! @param configuration Configuration used to initialize the Executor
        Executor(DeviceType device_type, const DeviceIds& ids,
                 const Configuration& configuration);

        //! @brief Tear down an Executor and free resources used by the
        //! Executor object
        ~Executor();

        //! Returns a vector of unique_ptr's to execution objects
        //! available on this instance of the Executor
        const ExecutionObjects& GetExecutionObjects() const;

        //! @brief Returns the number of devices of the specified type
        //! available for TI DL.
        //! @param  device_type DSP or EVE/DLA device
        //! @return number of devices available
        static uint32_t GetNumDevicesSupportingTIDL(DeviceType device_type);

        Executor(const Executor&) = delete;
        Executor& operator= (const Executor&) = delete;


    private:
        std::unique_ptr<ExecutorImpl> pimpl_m;
};


/*! @class ArgInfo
 *  @brief Describe input and output buffers required by ExecutionObjects
 */
class ArgInfo
{
    public:
        enum class DeviceAccess { R_ONLY=0, W_ONLY, RW };

        //! Enumerates the types of arguments represented by ArgInfo
        enum class Kind { BUFFER=0, SCALAR };

        //! Construct an ArgInfo object from a pointer to a chunk of memory
        //! and its size.
        ArgInfo(void *p, size_t size) :
            ptr_m(p), size_m(size),
            access_m(DeviceAccess::RW), kind_m(Kind::BUFFER) {}

        //! Construct an ArgInfo object from a pointer to a chunk of memory
        //! its size and kind
        ArgInfo(void *p, size_t size, Kind kind) :
            ptr_m(p), size_m(size), access_m(DeviceAccess::RW), kind_m(kind) {}

        //! @return Pointer to the buffer or scalar represented by ArgInfo
        void  *ptr()  const { return ptr_m; }

        //! @return The size of the buffer or scalar represented by ArgInfo
        size_t size() const { return size_m; }

        // Only used by tidl::Device
        Kind   kind() const { return kind_m; }
        bool   isLocal() const { return (ptr_m == nullptr); }

    private:
        void*        ptr_m;
        size_t       size_m;
        DeviceAccess access_m;
        Kind         kind_m;
};


extern "C" void  __free_ddr(void *ptr);
extern "C" void* __malloc_ddr(size_t s);

//! template typedef for unique_ptr with __free_ddr deleter
template<class T>
using up_malloc_ddr = std::unique_ptr<T, decltype(&__free_ddr)>;

//! __malloc_ddr wrapper - Bytes allocated determined by sizeof(T)
template<class T>
inline T* malloc_ddr()
{
    assert (std::is_pointer<T>::value == false);
    T* val =  reinterpret_cast<T *>(__malloc_ddr(sizeof(T)));
    assert (val != nullptr);
    return val;
}

//! __malloc_ddr wrapper - Bytes allocated passed as argument
template<class T>
inline T* malloc_ddr(size_t size)
{
    assert (std::is_pointer<T>::value == false);
    T* val = reinterpret_cast<T *>(__malloc_ddr(size));
    assert (val != nullptr);
    return val;
}

/*! @class Exception
 *  @brief Used to error reporting
 */
class Exception : public std::exception
{
    public:
        Exception() {}
        Exception(const std::string& error, const std::string& file,
                  const std::string& func, uint32_t line_no);
        Exception(int32_t errorCode, const std::string& file,
                  const std::string& func, uint32_t line_no);

        virtual ~Exception() {}

        //! @return String describing the error message and its location
        virtual const char* what() const noexcept;

    private:
        std::string message_m;
};

} // namespace tidl
