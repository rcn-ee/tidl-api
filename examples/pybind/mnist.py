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

import os
import argparse
import numpy as np

from tidl import DeviceId, DeviceType, Configuration, TidlError
from tidl import Executor, ExecutionObjectPipeline
from tidl import allocate_memory, free_memory


def main():
    """Read the configuration and run the network"""

    args = parse_args()

    config_file = '../test/testvecs/config/infer/tidl_config_mnist_lenet.txt'
    labels_file = '../test/testvecs/input/digits10_labels_10x1.y'

    configuration = Configuration()
    configuration.read_from_file(config_file)

    num_eve = Executor.get_num_devices(DeviceType.EVE)
    num_dsp = 0

    if num_eve == 0:
        print('MNIST network currently supported only on EVE')
        return

    run(num_eve, num_dsp, configuration, labels_file)

    return


DESCRIPTION = 'Run the mnist network on preprocessed input.'

def parse_args():
    """Parse input arguments"""

    parser = argparse.ArgumentParser(description=DESCRIPTION)
    args = parser.parse_args()

    return args

PIPELINE_DEPTH = 2

def run(num_eve, num_dsp, configuration, labels_file):
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

        if eve_device_ids:
            eve = Executor(DeviceType.EVE, eve_device_ids, configuration, 1)
            for i in range(eve.get_num_execution_objects()):
                eos.append(eve.at(i))

        if dsp_device_ids:
            dsp = Executor(DeviceType.DSP, dsp_device_ids, configuration, 1)
            for i in range(dsp.get_num_execution_objects()):
                eos.append(dsp.at(i))


        eops = []
        num_eos = len(eos)
        for j in range(PIPELINE_DEPTH):
            for i in range(num_eos):
                eops.append(ExecutionObjectPipeline([eos[i]]))

        allocate_memory(eops)

        # Open input, output files
        f_in = open(configuration.in_data, 'rb')
        f_labels = open(labels_file, 'rb')

        input_size = os.path.getsize(configuration.in_data)
        configuration.num_frames = int(input_size/(configuration.height *
                                                   configuration.width))

        print('TIDL API: processing {} input frames ...'.format(configuration.num_frames))

        num_eops = len(eops)
        num_errors = 0
        for frame_index in range(configuration.num_frames+num_eops):
            eop = eops[frame_index % num_eops]

            if eop.process_frame_wait():
                num_errors += process_output(eop, f_labels)

            if read_frame(eop, frame_index, configuration, f_in):
                eop.process_frame_start_async()


        f_in.close()
        f_labels.close()

        free_memory(eops)

        if num_errors == 0:
            print("mnist PASSED")
        else:
            print("mnist FAILED")

    except TidlError as err:
        print(err)

def read_frame(eo, frame_index, configuration, f_input):
    """Read a frame into the ExecutionObject input buffer"""

    if frame_index >= configuration.num_frames:
        return False

    # Read into the EO's input buffer
    arg_info = eo.get_input_buffer()
    bytes_read = f_input.readinto(arg_info)

    if bytes_read == 0:
        return False

    # TIDL library requires a minimum of 2 channels. Read image data into
    # channel 0. f_input.readinto will read twice as many bytes i.e. 2 input
    # digits. Seek back to avoid skipping inputs.
    f_input.seek((frame_index+1)*configuration.height*configuration.width)

    eo.set_frame_index(frame_index)

    return True

def process_output(eo, f_labels):
    """Display and check the inference result against labels."""

    maxval = 0
    maxloc = -1

    out_buffer = eo.get_output_buffer()
    output_array = np.asarray(out_buffer)
    for i in range(out_buffer.size()):
        if output_array[i] > maxval:
            maxval = output_array[i]
            maxloc = i

    print(maxloc)

    # Check inference result against label
    frame_index = eo.get_frame_index()
    f_labels.seek(frame_index)
    label = ord(f_labels.read(1))
    if maxloc != label:
        print('Error Expected {}, got {}'.format(label, maxloc))
        return 1

    return 0

if __name__ == '__main__':
    main()
