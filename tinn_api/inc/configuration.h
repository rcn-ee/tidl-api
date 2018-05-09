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

#pragma once

//! @file configuration.h

#include <string>
#include <iostream>

namespace tinn {

/*! @class Configuration
    @brief Specifies the configuration required for a network
*/
class Configuration
{
  public:

    //! Number of frames of input data (can be 0 if data is not read from file)
    int     numFrames;

    //! Height of the input frame
    int     inHeight;

    //! Width of the input frame
    int     inWidth;

    //! Number of channels in the input frame (e.g. 3 for BGR)
    int     inNumChannels;
    int     noZeroCoeffsPercentage;

    //! Pre-processing type applied to the input frame
    //! Specific to each network, can take values from 0 to 4, default is 0
    int     preProcType;

    //! layersGroupId in the network that the executor should work on
    int     layersGroupId;

    //! Force to run all layers, regardless of layersGroupId partitioning
    int     runFullNet;

    //! When set, inputs are taken from TIDL internal buffers that contain
    //! outputs of previous layersGroupId, instead of from user application
    int     enableInternalInput;

    //! Size of the TI DL per Execution Object heap
    size_t EXTMEM_HEAP_SIZE;

    //! Size of the heap used for paramter data
    size_t PARAM_HEAP_SIZE;

    //! @brief Location of the input file
    //! Can be empty if input data is provided by frameworks such as OpenCV.
    std::string inData;

    //! Location of the output file
    //! Can be empty if output data is consumed by frameworks such as OpenCV.
    std::string outData;

    //! Path to the TIDL network binary file
    std::string netBinFile;

    //! Path to the TIDL parameter binary file
    std::string paramsBinFile;

    //! Default constructor.
    Configuration();

    //! Validate the fields in the configuration object
    bool Validate() const;

    //! Print the configuration
    void Print(std::ostream& os = std::cout) const;

    //! Read a configuration from the specified file and validate
    bool ReadFromFile(const std::string& file_name);
};

}
