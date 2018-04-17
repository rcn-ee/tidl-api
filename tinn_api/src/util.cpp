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

using namespace tidl;

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

    delete b1;
    delete b2;

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

    delete b1;
    delete b2;

    return status;
}
