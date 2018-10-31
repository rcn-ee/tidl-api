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

extern "C" {
    #include <json-c/json.h>
}


ObjectClasses::ObjectClasses(const std::string& json_file) : num_classes_m(0)
{
    struct json_object *ocs = json_object_from_file(json_file.c_str());
    if (ocs != nullptr)
    {
        struct json_object *net_name, *objs, *obj, *label, *color;
        if (json_object_object_get_ex(ocs, "network", &net_name))
            network_name_m = json_object_get_string(net_name);

        if (json_object_object_get_ex(ocs, "objects", &objs))
        {
            num_classes_m = json_object_array_length(objs);
            for (unsigned int i = 0; i < num_classes_m; i++)
            {
                obj = json_object_array_get_idx(objs, i);

                std::string s_label;
                if (json_object_object_get_ex(obj, "label", &label))
                    s_label = json_object_get_string(label);
                else
                    s_label = "label" + std::to_string(i);

                unsigned char b, g, r;
                if (json_object_object_get_ex(obj, "color_bgr", &color))
                {
                    b = json_object_get_int(json_object_array_get_idx(color,0));
                    g = json_object_get_int(json_object_array_get_idx(color,1));
                    r = json_object_get_int(json_object_array_get_idx(color,2));
                }
                else
                    b = g = r = 255;

                classes_m.emplace_back(s_label, b, g, r);
            }
        }

        json_object_put(ocs);  // decrement refcount and free
    }

    classes_m.emplace_back("unknown", 0, 0, 0);  // sentinel
}

const ObjectClass& ObjectClasses::At(unsigned int index)
{
    if (index < num_classes_m)  return classes_m[index];
    else                        return classes_m[num_classes_m];
}

