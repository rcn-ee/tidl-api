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

""" Process each frame using a single ExecutionObject.
    Increase throughput by using multiple ExecutionObjects.
"""

import argparse

from tidl import DeviceId, DeviceType, Configuration, Executor, TidlError
from tidl import allocate_memory, free_memory, enable_time_stamps

from tidl_app_utils import read_frame, write_output, report_time


def main():
    """Read the configuration and run the network"""

    args = parse_args()

    # Heaps are sized for the j11_v2 network. Changing the network will
    # require updating network_heap_size and param_heap_size
    config_file = '../test/testvecs/config/infer/tidl_config_j11_v2.txt'

    configuration = Configuration()
    configuration.read_from_file(config_file)
    configuration.enable_api_trace = False
    configuration.num_frames = args.num_frames

    num_dsp = Executor.get_num_devices(DeviceType.DSP)
    num_eve = Executor.get_num_devices(DeviceType.EVE)

    if num_dsp == 0 and num_eve == 0:
        print('No TIDL API capable devices available')
        return

    enable_time_stamps("1eo_timestamp.log", 16)
    run(num_eve, num_dsp, configuration)

    return


DESCRIPTION = 'Process frames using all available Execution Objects. '\
              'Each ExecutionObject processes a single frame'

def parse_args():
    """Parse input arguments"""

    parser = argparse.ArgumentParser(description=DESCRIPTION)
    parser.add_argument('-n', '--num_frames',
                        type=int,
                        default=16,
                        help='Number of frames to process')
    args = parser.parse_args()

    return args


def run(num_eve, num_dsp, configuration):
    """ Run the network on the specified device type and number of devices"""

    print('Running network across {} EVEs, {} DSPs'.format(num_eve, num_dsp))

    dsp_device_ids = set([DeviceId.ID0, DeviceId.ID1,
                          DeviceId.ID2, DeviceId.ID3][0:num_dsp])
    eve_device_ids = set([DeviceId.ID0, DeviceId.ID1,
                          DeviceId.ID2, DeviceId.ID3][0:num_eve])

    # Heap sizes for this network determined using Configuration.showHeapStats
    configuration.param_heap_size = (3 << 20)
    configuration.network_heap_size = (20 << 20)

    try:
        print('TIDL API: performing one time initialization ...')

        # Collect all EOs from EVE and DSP executors
        eos = []

        if len(eve_device_ids) != 0:
            eve = Executor(DeviceType.EVE, eve_device_ids, configuration, 1)
            for i in range(eve.get_num_execution_objects()):
                eos.append(eve.at(i))

        if len(dsp_device_ids) != 0:
            dsp = Executor(DeviceType.DSP, dsp_device_ids, configuration, 1)
            for i in range(dsp.get_num_execution_objects()):
                eos.append(dsp.at(i))

        allocate_memory(eos)

        # Open input, output files
        f_in = open(configuration.in_data, 'rb')
        f_out = open(configuration.out_data, 'wb')


        print('TIDL API: processing input frames ...')

        num_eos = len(eos)
        for frame_index in range(configuration.num_frames+num_eos):
            execution_object = eos[frame_index % num_eos]

            if execution_object.process_frame_wait():
                report_time(execution_object)
                write_output(execution_object, f_out)

            if read_frame(execution_object, frame_index, configuration, f_in):
                execution_object.process_frame_start_async()


        f_in.close()
        f_out.close()

        free_memory(eos)
    except TidlError as err:
        print(err)


if __name__ == '__main__':
    main()
