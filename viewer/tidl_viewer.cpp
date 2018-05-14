/******************************************************************************
 * Copyright (c) 2017-18, Texas Instruments Incorporated - http://www.ti.com/
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
#include "tidl_viewer.h"
#include "dot_graph.h"

using namespace tidl::util;

bool tidl::util::PrintNetwork(const std::string& network_binary,
                              std::ostream& os)
{
    if (network_binary.empty())
        return false;

    sTIDL_Network_t net;
    bool status = ReadBinary(network_binary,
                             reinterpret_cast<char *>(&net),
                             sizeof(sTIDL_Network_t));
    if (!status)
    {
        std::cerr << "ERROR: Invalid network binary: "
                  << network_binary << std::endl;
        exit(EXIT_FAILURE);
    }

    printf("%3s  %-20s  %3s  %3s  %3s "
           " %3s  %3s  %3s  %3s  %3s  %3s  %3s  %3s %3s "
           " %5s  %5s  %5s  %5s  %5s  %5s  %5s  %5s\n",
            "#", "Name", "gId", "#i", "#o",
            "i0", "i1", "i2", "i3", "i4", "i5", "i6", "i7", "o",
            "#roi", "#ch", "h", "w", "#roi", "#ch", "h", "w");

    for (int i = 0 ; i < net.numLayers; i++)
    {
        printf("%3d, %-20s,",i,
                    TIDL_LayerString[net.TIDLLayers[i].layerType]);
        printf("%3d, %3d ,%3d ,",net.TIDLLayers[i].layersGroupId,
                                 net.TIDLLayers[i].numInBufs,
                                 net.TIDLLayers[i].numOutBufs);

        for (int j = 0; j < net.TIDLLayers[i].numInBufs; j++)
        {
          printf("%3d ,",net.TIDLLayers[i].inData[j].dataId);
        }
        for (int j = (net.TIDLLayers[i].numInBufs > 0 ?
              net.TIDLLayers[i].numInBufs : 0); j < 8; j++)
        {
          printf("  x ,");
        }
        printf("%3d ,",net.TIDLLayers[i].outData[0].dataId);
        for (int j = 0; j < 4; j++)
        {
          printf("%5d ,",net.TIDLLayers[i].inData[0].dimValues[j]);
        }
        for (int j = 0; j < 4; j++)
        {
          printf("%5d ,",net.TIDLLayers[i].outData[0].dimValues[j]);
        }
        printf("\n");
    }

    return true;
}


bool tidl::util::GenerateDotGraphForNetwork(const std::string& network_binary,
                                            const std::string& dot_file)
{
    if (network_binary.empty())
        return false;

    sTIDL_Network_t net;
    bool status = ReadBinary(network_binary,
                             reinterpret_cast<char *>(&net),
                             sizeof(sTIDL_Network_t));
    if (!status)
    {
        std::cerr << "ERROR: Invalid network binary: "
                  << network_binary << std::endl;
        exit(EXIT_FAILURE);
    }


    DotGraph g(net);
    g.Write(dot_file);

    return true;
}
