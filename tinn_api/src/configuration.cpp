/******************************************************************************
 * Copyright (c) 2017 Texas Instruments Incorporated - http://www.ti.com/
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

#include <string>
#include <fstream>

#include "configuration.h"

using namespace tidl;

void Configuration::Print(std::ostream &os) const
{
    os << "Configuration"
       << "\nFrame=      " << numFrames << " " << inWidth << "x"
                           << inHeight << "x" << inNumChannels
       << "\nInputFile                " << inData
       << "\nOutputFile               " << outData
       << "\nNetwork                  " << netBinFile
       << "\nParameters               " << paramsBinFile
       << "\nEO Heap Size (MB)        " << (EXTMEM_HEAP_SIZE >> 20)
       << "\nParameter heap size (MB) " << (PARAM_HEAP_SIZE >> 20)
       << "\n";
}

#include <sys/stat.h>
bool Configuration::Validate() const
{
    int errors = 0;

    if (inHeight == 0 || inWidth == 0)
    {
        std::cerr << "inHeight, inWidth must be > 0" << std::endl;
        errors++;
    }

    if (inNumChannels < 1)
    {
        std::cerr << "inNumChannels must be > 1" << std::endl;
        errors++;
    }

    struct stat buffer;
    if (stat(netBinFile.c_str(), &buffer) != 0)
    {
        std::cerr << "netBinFile not found: " << netBinFile << std::endl;
        errors++;
    }

    size_t paramsBinFileSize = 0;
    if (stat(paramsBinFile.c_str(), &buffer) != 0)
    {
        std::cerr << "paramsBinFile not found: " << paramsBinFile << std::endl;
        errors++;
    }
    else
        paramsBinFileSize = buffer.st_size;

    if (!inData.empty() && stat(inData.c_str(), &buffer) != 0)
    {
        std::cerr << "inData not found: " << inData << std::endl;
        errors++;
    }

    // Due to alignment, the parameter heap must be larger than the
    // parameter binary. Using 1.1 as a conservative factor.
    if (paramsBinFileSize > 0 &&
            (paramsBinFileSize * 1.1) > PARAM_HEAP_SIZE)
    {
        std::cerr << "Parameter binary file larger than paramter heap. "
                     "Increase Configuration::PARAM_HEAP_SIZE" << std::endl;
        errors++;
    }

    if (errors > 0)
        return false;

    return true;
}
