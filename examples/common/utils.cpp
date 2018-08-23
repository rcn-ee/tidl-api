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

#include <boost/format.hpp>
#include <cstring>

#include "utils.h"

using namespace tidl;

using boost::format;
using std::string;
using std::istream;
using std::ostream;
using std::vector;

static bool read_frame_helper(char* ptr, size_t size, istream& input_file);

bool ReadFrame(ExecutionObject*     eo,
               int                  frame_idx,
               const Configuration& configuration,
               std::istream&        input_file)
{
    if (frame_idx >= configuration.numFrames)
        return false;

    // Note: Frame index is used by the EO for debug messages only
    eo->SetFrameIndex(frame_idx);

    return read_frame_helper(eo->GetInputBufferPtr(),
                             eo->GetInputBufferSizeInBytes(),
                             input_file);
}

bool ReadFrame(ExecutionObjectPipeline* eop,
               int                      frame_idx,
               const Configuration&     configuration,
               std::istream&            input_file)
{
    if (frame_idx >= configuration.numFrames)
        return false;

    // Note: Frame index is used by the EOP for debug messages only
    eop->SetFrameIndex(frame_idx);

    return read_frame_helper(eop->GetInputBufferPtr(),
                             eop->GetInputBufferSizeInBytes(),
                             input_file);
}

bool read_frame_helper(char* ptr, size_t size, istream& input_file)
{
    assert (ptr != nullptr);
    assert (input_file.good());

    input_file.read(ptr, size);
    assert (input_file.good());

    if (input_file.eof())
        return false;

    // Wrap-around : if EOF is reached, start reading from the beginning.
    if (input_file.peek() == EOF)
        input_file.seekg(0, input_file.beg);

    if (input_file.good())
        return true;

    return false;
}


bool WriteFrame(const ExecutionObject* eo, ostream& output_file)
{
    output_file.write(eo->GetOutputBufferPtr(),
                      eo->GetOutputBufferSizeInBytes());
    assert(output_file.good() == true);

    if (output_file.good())
        return true;

    return false;
}

void ReportTime(const ExecutionObject* eo)
{
    double elapsed_host   = eo->GetHostProcessTimeInMilliSeconds();
    double elapsed_device = eo->GetProcessTimeInMilliSeconds();
    double overhead = 100 - (elapsed_device/elapsed_host*100);

    std::cout << format("frame[%3d]: Time on %s: %4.2f ms, host: %4.2f ms"
                        " API overhead: %2.2f %%\n")
                        % eo->GetFrameIndex() % eo->GetDeviceName()
                        % elapsed_device % elapsed_host % overhead;
}

void ReportTime(const ExecutionObjectPipeline* eop)
{
    double elapsed_host   = eop->GetHostProcessTimeInMilliSeconds();
    double elapsed_device = eop->GetProcessTimeInMilliSeconds();
    double overhead = 100 - (elapsed_device/elapsed_host*100);

    std::cout << format("frame[%3d]: Time on %s: %4.2f ms, host: %4.2f ms"
                        " API overhead: %2.2f %%\n")
                        % eop->GetFrameIndex() % eop->GetDeviceName()
                        % elapsed_device % elapsed_host % overhead;
}

// Compare output against reference output
bool CheckFrame(const ExecutionObject *eo, const char *ref_output)
{
    if (std::memcmp(static_cast<const void*>(ref_output),
               static_cast<const void*>(eo->GetOutputBufferPtr()),
               eo->GetOutputBufferSizeInBytes()) == 0)
        return true;

    return false;
}

bool CheckFrame(const ExecutionObjectPipeline *eop, const char *ref_output)
{
    if (std::memcmp(static_cast<const void*>(ref_output),
               static_cast<const void*>(eop->GetOutputBufferPtr()),
               eop->GetOutputBufferSizeInBytes()) == 0)
        return true;

    return false;
}


namespace tidl {
std::size_t GetBinaryFileSize (const std::string &F);
bool        ReadBinary        (const std::string &F, char* buffer, int size);
}

// Read file into a buffer.
const char* ReadReferenceOutput(const string& name)
{
    size_t size = GetBinaryFileSize(name);

    if (size == 0)
        return nullptr;

    char* buffer = new char[size];

    if (!buffer)
        return nullptr;

    if (!ReadBinary(name, buffer, size))
    {
        delete [] buffer;
        return nullptr;
    }

    return buffer;
}

// Allocate input and output memory for each EO
void AllocateMemory(const vector<ExecutionObject *>& eos)
{
    // Allocate input and output buffers for each execution object
    for (auto eo : eos)
    {
        size_t in_size  = eo->GetInputBufferSizeInBytes();
        size_t out_size = eo->GetOutputBufferSizeInBytes();
        void*  in_ptr   = malloc(in_size);
        void*  out_ptr  = malloc(out_size);
        assert(in_ptr != nullptr && out_ptr != nullptr);

        ArgInfo in  = { ArgInfo(in_ptr,  in_size)};
        ArgInfo out = { ArgInfo(out_ptr, out_size)};
        eo->SetInputOutputBuffer(in, out);
    }
}

// Free the input and output memory associated with each EO
void FreeMemory(const vector<ExecutionObject *>& eos)
{
    for (auto eo : eos)
    {
        free(eo->GetInputBufferPtr());
        free(eo->GetOutputBufferPtr());
    }
}

// Allocate input and output memory for each EOP
void AllocateMemory(const vector<ExecutionObjectPipeline *>& eops)
{
    // Allocate input and output buffers for each execution object
    for (auto eop : eops)
    {
        size_t in_size  = eop->GetInputBufferSizeInBytes();
        size_t out_size = eop->GetOutputBufferSizeInBytes();
        void*  in_ptr   = malloc(in_size);
        void*  out_ptr  = malloc(out_size);
        assert(in_ptr != nullptr && out_ptr != nullptr);

        ArgInfo in  = { ArgInfo(in_ptr,  in_size)};
        ArgInfo out = { ArgInfo(out_ptr, out_size)};
        eop->SetInputOutputBuffer(in, out);
    }
}

// Free the input and output memory associated with each EOP
void FreeMemory(const vector<ExecutionObjectPipeline *>& eops)
{
    for (auto eop : eops)
    {
        free(eop->GetInputBufferPtr());
        free(eop->GetOutputBufferPtr());
    }
}

