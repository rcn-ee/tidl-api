/******************************************************************************
 * Copyright (c) 2018 Texas Instruments Incorporated - http://www.ti.com/
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

#include "util.h"
#include <iostream>
#include <fstream>
#include <assert.h>
#include <chrono>
#include <mutex>
#include <cstring>
#include <memory>
#include "executor.h"

using namespace tidl;

using namespace std::chrono;

std::unique_ptr<TimeStamp> tidl_api_timestamps(nullptr);

bool tidl::EnableTimeStamps(const std::string& file, size_t num_frames)
{
    std::mutex m;
    std::lock_guard<std::mutex> guard(m);

    if (tidl_api_timestamps.get() != nullptr)
        return true;

    tidl_api_timestamps.reset(new TimeStamp(file, num_frames));
    if (tidl_api_timestamps.get() == nullptr)
        return false;

    return true;
}

TimeStamp::TimeStamp(const std::string& file, int num_entries):
                        num_entries_m(num_entries), file_m(file)
{
    entries_m = new Entry[num_entries_m];
    std::memset(entries_m, 0, sizeof(Entry)*num_entries_m);
}

void TimeStamp::Update(int frame_idx, EventKind k, int type, int id)
{
    int idx = frame_idx % num_entries_m;
    entries_m[idx].frame_idx = frame_idx;
    entries_m[idx].timestamp[k] = duration_cast<microseconds>
                 (high_resolution_clock::now().time_since_epoch()).count();

    if (k == EO1_PFSA_START)
    {
        entries_m[idx].eo1_id   = id;
        entries_m[idx].eo1_type = type;
    }
    else if (k == EO2_PFSA_START)
    {
        entries_m[idx].eo2_id   = id;
        entries_m[idx].eo2_type = type;
    }

}

TimeStamp::~TimeStamp()
{
    std::ofstream ofs;
    ofs.open(file_m, std::ofstream::out);

    for (int i=0; i < num_entries_m; i++)
        for (int j=0; j < EventKind::NUM_EVENTS; j++)
            if (entries_m[i].timestamp[j] != 0)
            {
                ofs << entries_m[i].frame_idx << ",";
                switch (j)
                {
                    case EOP_PFSA_START: ofs << "EOP:PFSA:Start"; break;
                    case EOP_PFSA_END:   ofs << "EOP:PFSA:End"; break;
                    case EOP_PFW_START:  ofs << "EOP:PFW:Start"; break;
                    case EOP_PFW_END:    ofs << "EOP:PFW:End"; break;
                    case EOP_RAN_START:  ofs << "EOP:RAN:Start"; break;
                    case EOP_RAN_END:    ofs << "EOP:RAN:End"; break;

                    case EO1_PFSA_START: ofs << "EO1:PFSA:Start"; break;
                    case EO1_PFSA_END:   ofs << "EO1:PFSA:End"; break;
                    case EO1_PFW_START:  ofs << "EO1:PFW:Start"; break;
                    case EO1_PFW_END:    ofs << "EO1:PFW:End"; break;

                    case EO2_PFSA_START: ofs << "EO2:PFSA:Start"; break;
                    case EO2_PFSA_END:   ofs << "EO2:PFSA:End"; break;
                    case EO2_PFW_START:  ofs << "EO2:PFW:Start"; break;
                    case EO2_PFW_END:    ofs << "EO2:PFW:End"; break;

                    default:             ofs << "UNKNOWN"; break;
                }
                ofs << "," << entries_m[i].timestamp[j];

                if (j == EO1_PFSA_START)
                {
                    ofs << "," << entries_m[i].eo1_type
                        << "," << entries_m[i].eo1_id;
                }
                else if (j == EO2_PFSA_START)
                {
                    ofs << "," << entries_m[i].eo2_type
                        << "," << entries_m[i].eo2_id;
                }


                ofs << std::endl;
            }

    ofs.close();

    delete [] entries_m;
}


void tidl::RecordEvent(int frame_idx, TimeStamp::EventKind k,
                       int eo_type, int eo_id)
{
    TimeStamp* t = tidl_api_timestamps.get();
    if (t)
        t->Update(frame_idx, k, eo_type, eo_id);
}


std::size_t tidl::GetBinaryFileSize(const std::string &F)
{
    std::ifstream is;
    is.open (F, std::ios::binary );

    if (!is.good())
    {
        std::cout << "ERROR: File read failed for " << F << std::endl;
        return 0;
    }

    is.seekg (0, std::ios::end);
    size_t length = is.tellg();
    is.close();

    return length;
}


bool tidl::ReadBinary(const std::string &F, char* buffer, int size)
{
    std::ifstream is;
    is.open (F, std::ios::binary );

    if (!is.good())
    {
        std::cout << "ERROR: File read failed for " << F << std::endl;
        return false;
    }

    is.seekg (0, std::ios::end);
    int length = is.tellg();

    if (length != size)
    {
        std::cout << length << " != " << size << std::endl;
        is.close();
        return false;
    }

    is.seekg (0, std::ios::beg);
    is.read (buffer, length);
    is.close();

    return true;
}

bool tidl::CompareFiles(const std::string &F1, const std::string &F2)
{
    std::size_t s1 = GetBinaryFileSize(F1);
    std::size_t s2 = GetBinaryFileSize(F2);

    if (s1 != s2)
        return false;

    char *b1 = new char[s1];
    char *b2 = new char[s2];

    ReadBinary(F1, b1, s1);
    ReadBinary(F2, b2, s2);

    int errors = 0;
    for (size_t i=0; i < s1; i++)
        if (b1[i] != b2[i])
        {
            std::cout << "Error at " << i << " " <<
                         (int)b1[i] << " != " << (int)b2[i];
            std::cout << std::endl;
            errors++;

            if (errors > 10)
                break;
        }

    delete[] b1;
    delete[] b2;

    if (errors == 0) return true;

    return false;
}

bool tidl::CompareFrames(const std::string &F1, const std::string &F2,
                         int numFrames, int width, int height)
{
    bool status = true;

    std::size_t s1 = GetBinaryFileSize(F1);
    std::size_t s2 = GetBinaryFileSize(F2);

    char *b1 = new char[s1];
    char *b2 = new char[s2];

    ReadBinary(F1, b1, s1);
    ReadBinary(F2, b2, s2);

    for (int f=0; f < numFrames; f++)
    {
        int errors = 0;
        int frame_offset = f*width*height;
        std::cout << "Comparing frame: " << f << std::endl;
        for (int h=0; h < height; h++)
        {
            for (int w=0; w < width; w++)
            {
                size_t index = frame_offset+(h*width)+w;
                assert (index < s1 && index < s2);
                if (b1[index] != b2[index])
                {
                    status = false;
                    std::cout << "Error at " << index << " " <<
                                 (int)b1[index] << " != " << (int)b2[index];
                    std::cout << std::endl;
                    errors++;

                }
                if (errors > 10) break;
            }
            if (errors > 10) break;
        }
    }

    delete[] b1;
    delete[] b2;

    return status;
}
