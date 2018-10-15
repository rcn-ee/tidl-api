/******************************************************************************
 * Copyright (c) 2018, Texas Instruments Incorporated - http://www.ti.com/
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

#pragma once

#include <string>
#include <cstddef>
#include "configuration.h"

namespace tidl {

std::size_t GetBinaryFileSize (const std::string &F);
bool        ReadBinary        (const std::string &F, char* buffer, int size);
bool        CompareFiles      (const std::string &F1, const std::string &F2);
bool        CompareFrames(const std::string &F1, const std::string &F2,
                         int numFrames, int width, int height);

class TimeStamp
{
    public:
        enum EventKind { EOP_PFSA_START=0, EOP_PFSA_END,
                         EOP_PFW_START,    EOP_PFW_END,
                         EOP_RAN_START,    EOP_RAN_END,
                         EO1_PFSA_START,   EO1_PFSA_END,
                         EO1_PFW_START,    EO1_PFW_END,
                         EO2_PFSA_START,   EO2_PFSA_END,
                         EO2_PFW_START,    EO2_PFW_END,
                         NUM_EVENTS };
        struct Entry
        {
            unsigned long long frame_idx;
            unsigned long long timestamp[EventKind::NUM_EVENTS];
        };


        TimeStamp(const std::string& file, int num_entries);
        ~TimeStamp();
        void Update(int frame_idx, EventKind k);

    private:
        Entry*            entries_m;
        const int         num_entries_m;
        const std::string file_m;
};


void RecordEvent(int frame_idx, TimeStamp::EventKind k);

} // namespace tidl
