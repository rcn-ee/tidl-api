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

#include <iostream>
#include "imgutil.h"

#include "opencv2/imgproc.hpp"
using namespace tidl;
using namespace cv;

bool tidl::imgutil::PreprocessImage(Mat& image, char *ptr,
                                    const Configuration& c)
{
    int num_channels  = c.inNumChannels;
    int output_width  = c.inWidth;
    int output_height = c.inHeight;
    int pitch         = c.inWidth;
    int chOffset      = c.inWidth * c.inHeight;
    int preProcType   = c.preProcType;

    bool enableMeanSub = false;
    Mat tempImage;
    int32_t meanValues[num_channels];

    if(preProcType == 0) // Caffe-Jacinto Models
    {
        int32_t half_the_width  =  256/ 2;
        int32_t half_the_height =  256/ 2;

        int32_t startX  = half_the_width - output_width/2;
        int32_t startY  = half_the_height - output_height/2;

        cv::resize(image, tempImage, Size(256,256), 0,0,cv::INTER_AREA);

        cv::Rect myROI(startX, startY, output_width, output_height);
        image = tempImage(myROI);

        enableMeanSub = false;
    }
    else if (preProcType == 1) // Caffe Models , eg : SqueezeNet
    {
        int32_t half_the_width  =  256/ 2;
        int32_t half_the_height =  256/ 2;

        int32_t startX  = half_the_width - output_width/2;
        int32_t startY  = half_the_height - output_height/2;

        cv::resize(image, tempImage, Size(256,256), 0,0,cv::INTER_AREA);

        cv::Rect myROI(startX, startY, output_width, output_height);
        image = tempImage(myROI);

        enableMeanSub = true;
        meanValues[0] = 104;
        meanValues[1] = 117;
        meanValues[2] = 123;

    }
    else if (preProcType == 2) // mobileNet, inceptionNet
    {
        cv::cvtColor(image, image, CV_BGR2RGB);
        float factor  = 0.875;
        int32_t orgWidth  = image.size[1];
        int32_t orgHeight = image.size[0];

        int32_t crop_width    =  orgWidth*factor;
        int32_t crop_height    =  orgHeight*factor;
        int32_t half_the_width  = orgWidth/ 2;
        int32_t half_the_height = orgHeight / 2;

        int32_t startX  = half_the_width - crop_width/2;
        int32_t startY  = half_the_height - crop_height/2;

        cv::Rect myROI(startX, startY, crop_width, crop_height);
        tempImage = image(myROI);

        cv::resize(tempImage, image, Size(output_width,output_height),
                   0, 0, CV_INTER_AREA);

        enableMeanSub = true;
        meanValues[0] = 128;
        meanValues[1] = 128;
        meanValues[2] = 128;
    }
    else if (preProcType == 3)
    {
        cv::cvtColor(image, image, CV_BGR2RGB);
        int32_t half_the_width  =  32/ 2;
        int32_t half_the_height =  32/ 2;

        int32_t startX  = half_the_width - output_width/2;
        int32_t startY  = half_the_height - output_height/2;


        cv::resize(image, tempImage, Size(32,32), 0,0,cv::INTER_AREA);
        cv::Rect myROI(startX, startY, output_width, output_height);
        image = tempImage(myROI);
        enableMeanSub = false;
    }
    else if (preProcType == 4)
    {
        cv::resize(image, tempImage, Size(output_width,output_height),
                   0, 0, cv::INTER_AREA);
        enableMeanSub = false;
    }
    else
    {
        std::cerr << "Unsupported preProcType : " << preProcType << std::endl;
        return false;
    }

    if (image.channels() > 3)
        return false;

    if (image.total() != (unsigned int) (output_height * output_width))
        return false;

    Mat spl[num_channels];
    split(image,spl);

    for(int c = 0; c < num_channels; c++)
    {
        const unsigned char* data = spl[c].ptr();

        for(int rows = 0; rows < output_height; rows++)
            for(int cols = 0; cols < output_width; cols++)
            {
                int32_t in =  data[rows*output_width + cols];

                if(enableMeanSub)
                {
                    in -= meanValues[c];

                    if(in > 127)  in  = 127;
                    if(in < -128) in = -128;
                }

                ptr[c*chOffset + rows*pitch + cols] = in;
            }
    }

    return true;
}
