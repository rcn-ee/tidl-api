#!/usr/bin/python3

# Copyright (c) 2018 Texas Instruments Incorporated - http://www.ti.com/
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of Texas Instruments Incorporated nor the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.


from tidl import DeviceId
from tidl import DeviceType
from tidl import Configuration
from tidl import Executor
from tidl import TidlError

import tidl

def read_frame(eo, frame_index, c, f):
    """Read a frame into the ExecutionObject input buffer"""

    if (frame_index >= c.num_frames):
        return False

    # Read into the EO's input buffer
    arg_info = eo.get_input_buffer()
    bytes_read = f.readinto(arg_info)

    if (bytes_read != arg_info.size()):
        print("Expected {} bytes, read {}".format(size, bytes_read))
        return False

    if (len(f.peek(1)) == 0):
        f.seek(0)

    eo.set_frame_index(frame_index)

    return True

def write_output(eo, f):
    """Write the output buffer to file"""

    arg_info = eo.get_output_buffer()
    f.write(arg_info)

def report_time(eo):
    """Report execution time on device"""

    elapsed_device = eo.get_process_time_in_ms()

    # https://pyformat.info/
    print('frame{:3d}: Time on {}: {:4.2f} ms '.format(eo.get_frame_index(),
                                                       eo.get_device_name(),
                                                       elapsed_device))
