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

#pragma once

//! @file configuration.h

#include <string>
#include <map>
#include <iostream>

namespace tidl {

/*! @class Configuration
    @brief Specifies the configuration required for a network

    The Configuration object is used to specify various parameters required
    for network execution. Applications can directly initialize fields in an
    instance of Configuration or use the ReadFromFile method to read the
    configuration from a file.
*/

class Configuration
{
  public:

    //! Number of frames of input data (can be 0 if data is not read from file)
    int     numFrames;

    //! Height of the input image. Used by the API, must be specified.
    int     inHeight;

    //! Width of the input image. Used by the API, must be specified.
    int     inWidth;

    //! Number of channels in the input frame (e.g. 3 for BGR)
    //! Used by the API, must be specified.
    int     inNumChannels;

    //! @private
    int     noZeroCoeffsPercentage;

    //! Pre-processing type applied to the input frame
    //! Specific to each network, can take values from 0 to 4, default is 0
    //! 0 -> Caffe-Jacinto models
    //! 1 -> Caffe models (SqueezeNet)
    //! 2 -> TensorFlow (Inception, MobileNet)
    //! 3 -> CIFAR 10
    //! 4 -> JdetNet
    int     preProcType;

    //! Force to run all layers, regardless of layersGroupId partitioning
    bool    runFullNet;

    //! @brief Size of the device side network heap
    //! This heap is used for allocating memory required to
    //! run the network on the device. One per Execution Object.
    size_t NETWORK_HEAP_SIZE;

    //! @brief Size of the device side heap used for parameter data.
    //! The size depends on the size of the parameter binary file. The
    //! constructor for ``Configuration`` sets PARAM_HEAP_SIZE to 9MB.
    //! There is one parameter heap for each instance of ``Executor`` .
    size_t PARAM_HEAP_SIZE;

    //! @brief Path to the input image file.
    //! This field is not used by the TIDL API itself. It can be used by
    //! applications to load an input image into a buffer. Can be empty if
    //! the application uses frameworks such as OpenCV to read images. Refer
    //! examples/test/main.cpp for usage.
    std::string inData;

    //! @brief Path to the output image file.
    //! This field is not used by the TIDL API itself. It can be used by
    //! applications to specify a name for the output file. Can be empty
    //! if the application uses frameworks such as OpenCV to read images.
    //! Refer examples/test/main.cpp for usage.
    std::string outData;

    //! Path to the TIDL network binary file.
    //! Used by the API, must be specified.
    std::string netBinFile;

    //! Path to the TIDL parameter binary file
    //! Used by the API, must be specified.
    std::string paramsBinFile;

    //! Map of layer index to layer group id. Used to override layer group
    //! assigment for layers. Any layer not specified in this map will
    //! retain its existing mapping.
    std::map<int, int> layerIndex2LayerGroupId;

    //! Enable tracing of output buffers associated with each layer
    bool enableOutputTrace;

    //! Debug - Generates a trace of host and device function calls
    bool enableApiTrace;

    //! Debug - Shows total size of PARAM and NETWORK heaps. Also shows bytes
    //! available after all allocations. Can be used to adjust the heap
    //! size.
    bool showHeapStats;

    int    quantHistoryParam1;
    int    quantHistoryParam2;
    int    quantMargin;

    //! Default constructor.
    Configuration();

    //! Validate the fields in the configuration object
    bool Validate() const;

    //! Debug - Print the configuration.
    void Print(std::ostream& os = std::cout) const;

    //! Read a configuration from the specified file and validate
    bool ReadFromFile(const std::string& file_name);

};

}
