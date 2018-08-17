/******************************************************************************
 * Copyright (c) 2018, Texas Instruments Incorporated - http://www.ti.com/
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *       * Redistributions of source code must retain the above copyright
 *         notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *         notice, this list of conditions and the following disclaimer in the
 *         documentation and/or other materials provided with the distribution.
 *       * Neither the name of Texas Instruments Incorporated nor the
 *         names of its contributors may be used to endorse or promote products
 *         derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *   THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "object_classes.h"

object_class_table_t jseg21_table =
{
    5,
    {
        { "background",       { 127, 127, 127 } },
        { "road",             {   0, 255,   0 } },
        { "pedestrian",       { 255,   0,   0 } },
        { "road sign",        { 255, 255,   0 } },
        { "vehicle",          {   0,   0, 255 } },
        { "unknown",          {   0,   0,   0 } }    /* guard */
    }
};

object_class_table_t jdetnet_table =
{
    4,
    {
        { "some class 1",     { 255,   0, 255 } },
        { "pedestrian",       { 255,   0,   0 } },
        { "road sign",        { 255, 255,   0 } },
        { "vehicle",          {   0,   0, 255 } },
        { "unknown",          {   0,   0,   0 } }    /* guard */
    }
};

object_class_table_t* GetObjectClassTable(std::string &config)
{
     if (config.compare(0, 6, "jseg21") == 0)  return &jseg21_table;
     else if (config == "jdetnet")             return &jdetnet_table;
     else                                      return nullptr;
}

object_class_t* GetObjectClass(object_class_table_t *table, int index)
{
    if (index < 0 || (unsigned int)index >= table->num_classes)  index = table->num_classes;
    return & (table->classes[index]);
}

