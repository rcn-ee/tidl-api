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

using namespace tinn;

bool tinn::imgutil::PreProcImage(
                  Mat& image, char *ptr, int16_t roi, int16_t n,
                  int16_t width, int16_t height, int16_t pitch,
                  int32_t chOffset, int32_t frameCount, int32_t preProcType)
{
  int32_t  i0, i1, i2, i3;
  Mat spl[3];
  int32_t enableMeanSub = 0;
  Mat tempImage;
  int32_t meanVlaues[3];

  if(preProcType == 0) // Caffe-Jacinto Models
  {
   int32_t crop_width    =  width;
   int32_t crop_height    =  height;
   int32_t half_the_width  =  256/ 2;
   int32_t half_the_height =  256/ 2;

   int32_t startX  = half_the_width -crop_width/2;
   int32_t startY  = half_the_height -crop_height/2;

   cv::Rect myROI(startX,
   startY,
   crop_width,
   crop_height);
   cv::resize(image, tempImage, Size(256,256), 0,0,cv::INTER_AREA);
   image = tempImage(myROI);
   enableMeanSub = 0;
  }
  else if (preProcType == 1) // Caffe Models , eg : SqueezeNet
  {

   int32_t crop_width    =  width;
   int32_t crop_height    =  height;
   int32_t half_the_width  =  256/ 2;
   int32_t half_the_height =  256/ 2;

   int32_t startX  = half_the_width -crop_width/2;
   int32_t startY  = half_the_height -crop_height/2;

   cv::Rect myROI(startX,
   startY,
   crop_width,
   crop_height);
   cv::resize(image, tempImage, Size(256,256), 0,0,cv::INTER_AREA);
   image = tempImage(myROI);
   enableMeanSub = 1;
   meanVlaues[0] = 104;
   meanVlaues[1] = 117;
   meanVlaues[2] = 123;

  }
  else if (preProcType == 2)
  {
   cv::cvtColor(image, image, CV_BGR2RGB);
   float factor  = 0.875;
   int32_t orgWidth  = image.size[1];
   int32_t orgHeight = image.size[0];

   int32_t crop_width    =  orgWidth*factor;
   int32_t crop_height    =  orgHeight*factor;
   int32_t half_the_width  = orgWidth/ 2;
   int32_t half_the_height = orgHeight / 2;

   int32_t startX  = half_the_width -crop_width/2;
   int32_t startY  = half_the_height -crop_height/2;

   cv::Rect myROI(startX,
   startY,
   crop_width,
   crop_height);
   tempImage = image(myROI);
   cv::resize(tempImage, image, Size(width,height), 0,0,CV_INTER_AREA);
   enableMeanSub = 1;
   meanVlaues[0] = 128;
   meanVlaues[1] = 128;
   meanVlaues[2] = 128;
  }
  else if (preProcType == 3)
  {
   cv::cvtColor(image, image, CV_BGR2RGB);
   int32_t crop_width    =  width;
   int32_t crop_height    =  height;
   int32_t half_the_width  =  32/ 2;
   int32_t half_the_height =  32/ 2;

   int32_t startX  = half_the_width -crop_width/2;
   int32_t startY  = half_the_height -crop_height/2;

   cv::Rect myROI(startX,
   startY,
   crop_width,
   crop_height);
   cv::resize(image, tempImage, Size(32,32), 0,0,cv::INTER_AREA);
   image = tempImage(myROI);
   enableMeanSub = 0;
  }
  else if (preProcType == 4)
  {
   //cv::cvtColor(image, image, CV_BGR2RGB);
   int32_t crop_width    =  width;
   int32_t crop_height    =  height;
   int32_t half_the_width  =  width/ 2;
   int32_t half_the_height =  height/ 2;

   int32_t startX  = half_the_width -crop_width/2;
   int32_t startY  = half_the_height -crop_height/2;

   cv::Rect myROI(startX,
   startY,
   crop_width,
   crop_height);
   cv::resize(image, tempImage, Size(width,height), 0,0,cv::INTER_AREA);
   image = tempImage(myROI);
   enableMeanSub = 0;
  }
  else
  {
    std::cerr << "Unsupported preProcType : " << preProcType << std::endl;
    return false;
  }

  if(image.channels() > 3)
  {
   return false;
  }
  if(image.total() != (unsigned int) (height * width))
  {
   return false;
  }
  int size = (int)image.total()*image.channels();
  uint8_t * data = (uint8_t*)malloc(size);

  if(data == NULL)
  {
   return false;
  }

  split(image,spl);
  for(i0 = 0; i0 < image.channels(); i0++)
  {
   std::memcpy(&data[i0*((int)image.total())],spl[i0].data,((int)image.total()) * sizeof(uint8_t));
  }
  for(i2 = 0; i2 < roi; i2++)
  {
   for(i0 = 0; i0 < n; i0++)
   {
    for(i1 = 0; i1 < height; i1++)
    {
      for(i3 = 0; i3 < width; i3++)
      {
       int32_t in;

       if(enableMeanSub)
       {
        if(n != 1)
        {
          in =  data[i2*n*width*height + i0*width*height+ i1*width + i3] - meanVlaues[i0];
        }
        else
        {
          in =  data[i2*1*width*height + i1*width + i3] - meanVlaues[i0];
        }

        if(in > 127)  in  = 127;
        if(in < -128) in = -128;
       }
       else
       {

        if(n != 1)
        {
          in =  data[i2*n*width*height + i0*width*height+ i1*width + i3];
        }
        else
        {
          in =  data[i2*1*width*height + i1*width + i3];
        }
       }

       ptr[i2*n*chOffset + i0*chOffset + i1*pitch + i3] = in;
      }
    }
   }
  }

  std::free(data);
  return true;
}
