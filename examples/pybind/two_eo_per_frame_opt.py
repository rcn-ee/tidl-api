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


import argparse
from tidl import DeviceId, DeviceType, Configuration, TidlError
from tidl import Executor, ExecutionObjectPipeline
from tidl import allocate_memory, free_memory, enable_time_stamps

from tidl_app_utils import read_frame, write_output


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

    # Heap sizes for this network determined using Configuration.showHeapStats
    configuration.param_heap_size = (3 << 20)
    configuration.network_heap_size = (20 << 20)

    num_dsp = Executor.get_num_devices(DeviceType.DSP)
    num_eve = Executor.get_num_devices(DeviceType.EVE)

    if num_dsp == 0 or num_eve == 0:
        print('This example required EVEs and DSPs.')
        return

    enable_time_stamps("2eo_opt_timestamp.log", 16)
    run(num_eve, num_dsp, configuration)

# Run layer group 1 on EVE, 2 on DSP
EVE_LAYER_GROUP_ID = 1
DSP_LAYER_GROUP_ID = 2


def run(num_eve, num_dsp, c):
    """ Run the network on the specified device type and number of devices"""

    print('Running on {} EVEs, {} DSPs'.format(num_eve, num_dsp))

    dsp_device_ids = set([DeviceId.ID0, DeviceId.ID1,
                          DeviceId.ID2, DeviceId.ID3][0:num_dsp])
    eve_device_ids = set([DeviceId.ID0, DeviceId.ID1,
                          DeviceId.ID2, DeviceId.ID3][0:num_eve])

    c.layer_index_to_layer_group_id = {12:DSP_LAYER_GROUP_ID,
                                       13:DSP_LAYER_GROUP_ID,
                                       14:DSP_LAYER_GROUP_ID}

    try:
        print('TIDL API: performing one time initialization ...')

        eve = Executor(DeviceType.EVE, eve_device_ids, c, EVE_LAYER_GROUP_ID)
        dsp = Executor(DeviceType.DSP, dsp_device_ids, c, DSP_LAYER_GROUP_ID)

        num_eve_eos = eve.get_num_execution_objects()
        num_dsp_eos = dsp.get_num_execution_objects()

        # On AM5749, create a total of 4 pipelines (EOPs):
        # EOPs[0] : { EVE1, DSP1 }
        # EOPs[1] : { EVE1, DSP1 } for double buffering
        # EOPs[2] : { EVE2, DSP2 }
        # EOPs[3] : { EVE2, DSP2 } for double buffering
        PIPELINE_DEPTH = 2

        eops = []
        num_pipe = max(num_eve_eos, num_dsp_eos)
        for i in range(num_pipe):
            for i in range(PIPELINE_DEPTH):
                eops.append(ExecutionObjectPipeline([eve.at(i % num_eve_eos),
                                                     dsp.at(i % num_dsp_eos)]))

        allocate_memory(eops)

        # Open input, output files
        f_in = open(c.in_data, 'rb')
        f_out = open(c.out_data, 'wb')


        print('TIDL API: processing input frames ...')

        num_eops = len(eops)
        for frame_index in range(c.num_frames+num_eops):
            eop = eops[frame_index % num_eops]

            if eop.process_frame_wait():
                write_output(eop, f_out)

            if read_frame(eop, frame_index, c, f_in):
                eop.process_frame_start_async()


        f_in.close()
        f_out.close()

        free_memory(eops)
    except TidlError as err:
        print(err)


DESCRIPTION = 'Process frames using ExecutionObjectPipeline. '\
              'Each ExecutionObjectPipeline processes a single frame. '\
              'The example also uses double buffering on the EOPs to '\
              'hide frame read overhead.'

def parse_args():
    """Parse input arguments"""

    parser = argparse.ArgumentParser(description=DESCRIPTION)
    parser.add_argument('-n', '--num_frames',
                        type=int,
                        default=16,
                        help='Number of frames to process')
    args = parser.parse_args()

    return args


if __name__ == '__main__':
    main()
