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

#include <string>
#include <fstream>

#include "configuration.h"
#include "parameters.h"

using namespace tidl;

Configuration::Configuration(): numFrames(0), inHeight(0), inWidth(0),
                     inNumChannels(0),
                     noZeroCoeffsPercentage(100),
                     preProcType(0),
                     runFullNet(false),
                     NETWORK_HEAP_SIZE(internal::DEFAULT_NETWORK_HEAP_SIZE),
                     PARAM_HEAP_SIZE(internal::DEFAULT_PARAM_HEAP_SIZE),
                     enableOutputTrace(false),
                     enableApiTrace(false),
                     showHeapStats(false),
                     quantHistoryParam1(20),
                     quantHistoryParam2(5),
                     quantMargin(0),
                     isSubgraphCfg(false),
                     inConvType(),
                     inIsSigned(),
                     inScaleF2Q(),
                     inIsNCHW(),
                     outConvType(),
                     outIsSigned(),
                     outScaleF2Q(),
                     outIsNCHW()
{
}

void Configuration::Print(std::ostream &os) const
{
    os << "Configuration"
       << "\nFrame=      " << numFrames << " " << inWidth << "x"
                           << inHeight << "x" << inNumChannels
       << "\nPreProcType              " << preProcType
       << "\nRunFullNet               " << runFullNet
       << "\nInputFile                " << inData
       << "\nOutputFile               " << outData
       << "\nNetwork                  " << netBinFile
       << "\nParameters               " << paramsBinFile
       << "\nEO Heap Size (MB)        " << (NETWORK_HEAP_SIZE >> 20)
       << "\nParameter heap size (MB) " << (PARAM_HEAP_SIZE >> 20)
       << "\n";
}

#include <sys/stat.h>
bool Configuration::Validate() const
{
    int errors = 0;
    struct stat buffer;

    if (! isSubgraphCfg)
    {
        if (inHeight == 0 || inWidth == 0)
        {
            std::cerr << "cfg: inHeight, inWidth must be > 0" << std::endl;
            errors++;
        }

        if (inNumChannels < 1)
        {
            std::cerr << "cfg: inNumChannels must be > 1" << std::endl;
            errors++;
        }

        if (!inData.empty() && stat(inData.c_str(), &buffer) != 0)
        {
            std::cerr << "cfg: inData not found: " << inData << std::endl;
            errors++;
        }
    }
    else
    {
        if (inConvType.size() == 0 || inIsSigned.size() == 0 ||
            inScaleF2Q.size() == 0 || inIsNCHW.size() == 0 ||
            outConvType.size() == 0 || outIsSigned.size() == 0 ||
            outScaleF2Q.size() == 0 || outIsNCHW.size() == 0)
        {
            std::cerr << "cfg: subgraph data info not found" << std::endl;
            errors++;
        }

        if (inConvType.size() != inIsNCHW.size() ||
            outConvType.size() != outIsNCHW.size())
        {
            std::cerr << "cfg: Mismatching subgraph data info" << std::endl;
            errors++;
        }
    }

    if (stat(netBinFile.c_str(), &buffer) != 0)
    {
        std::cerr << "cfg: netBinFile not found: " << netBinFile << std::endl;
        errors++;
    }

    if (stat(paramsBinFile.c_str(), &buffer) != 0)
    {
        std::cerr << "cfg: paramsBinFile not found: " << paramsBinFile
                  << std::endl;
        errors++;
    }

    if (errors > 0)
        return false;

    return true;
}
