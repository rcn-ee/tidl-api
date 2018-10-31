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

""" Layer output

Illustrates writing the outputs of intermediate layers in the network to file.
"""

import argparse

from tidl import DeviceId, DeviceType, Configuration, Executor, TidlError
from tidl import allocate_memory, free_memory

from tidl_app_utils import read_frame


def main():
    """ Parse arguments, read configuration and run network"""

    parser = argparse.ArgumentParser(description=
                                     'Dump output of each network layer to file.')
    parser.add_argument(
        '-c', '--config_file',
        default='../test/testvecs/config/infer/tidl_config_j11_v2.txt',
        help='Path to TIDL config file')

    args = parser.parse_args()

    # Run network for 1 frame since we interested in intermediate layer outputs
    num_frames = 1

    # Read configuration from file
    configuration = Configuration()
    configuration.read_from_file(args.config_file)
    configuration.enable_layer_dump = True
    configuration.num_frames = num_frames

    num_dsp = Executor.get_num_devices(DeviceType.DSP)
    num_eve = Executor.get_num_devices(DeviceType.EVE)

    if num_dsp == 0 and num_eve == 0:
        print('No TIDL API capable devices available')
        return

    if num_eve > 0:
        device_type = DeviceType.EVE
    else:
        device_type = DeviceType.DSP

    # Since we are dumping layer outputs, just run on one device
    run(device_type, 1, configuration)


def run(device_type, num_devices, configuration):
    """ Run the network on a single device and dump output of each layer"""

    print('Running network on {} {}'.format(num_devices, device_type))

    device_ids = set([DeviceId.ID0])

    try:
        print('TIDL API: performing one time initialization ...')

        executor = Executor(device_type, device_ids, configuration, 1)

        # Collect all EOs from EVE and DSP executors
        eos = []
        for i in range(executor.get_num_execution_objects()):
            eos.append(executor.at(i))

        allocate_memory(eos)

        # Open input, output files
        f_in = open(configuration.in_data, 'rb')


        print('TIDL API: processing input frames ...')

        num_eos = len(eos)
        for frame_index in range(configuration.num_frames+num_eos):
            execution_object = eos[frame_index % num_eos]

            if execution_object.process_frame_wait():
                execution_object.write_layer_outputs_to_file()

            if read_frame(execution_object, frame_index, configuration, f_in):
                execution_object.process_frame_start_async()

        f_in.close()

        free_memory(eos)
    except TidlError as err:
        print(err)


if __name__ == '__main__':
    main()
