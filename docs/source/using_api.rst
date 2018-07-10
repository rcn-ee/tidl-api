.. _using-tidl-api:

******************
Using the TIDL API
******************

This example illustrates using the TIDL API to offload deep learning network processing from a Linux application to the C66x DSPs or EVEs on AM57x devices. The API consists of three classes: ``Configuration``, ``Executor`` and ``ExecutionObject``.

Step 1
======

Determine if there are any TIDL capable devices on the AM57x SoC:

.. code-block:: c++

    uint32_t num_eve = Executor::GetNumDevices(DeviceType::EVE);
    uint32_t num_dsp = Executor::GetNumDevices(DeviceType::DSP);

.. note::
    By default, the OpenCL runtime is configured with sufficient global memory 
    (via CMEM) to offload TIDL networks to 2 OpenCL devices. On devices where
    ``Executor::GetNumDevices`` returns 4 (E.g. AM5729 with 4 EVE OpenCL
    devices) the amount of memory available to the runtime must be increased. 
    Refer :ref:`opencl-global-memory` for details

Step 2
======
Create a Configuration object by reading it from a file or by initializing it directly. The example below parses a configuration file and initializes the Configuration object. See ``examples/test/testvecs/config/infer`` for examples of configuration files.

.. code::

    Configuration configuration;
    bool status = configuration.ReadFromFile(config_file);

.. note::
    Refer `Processor SDK Linux Software Developer's Guide`_ for creating TIDL network and parameter binary files from TensorFlow and Caffe.

Step 3
======
Create an Executor with the appropriate device type, set of devices and a configuration. In the snippet below, an Executor is created on 2 EVEs.

.. code-block:: c++

        DeviceIds ids = {DeviceId::ID0, DeviceId::ID1};
        Executor executor(DeviceType::EVE, ids, configuration);

Step 4
======
Get the set of available ExecutionObjects and allocate input and output buffers for each ExecutionObject.

.. code-block:: c++

        const ExecutionObjects& execution_objects = executor.GetExecutionObjects();
        int num_eos = execution_objects.size();

        // Allocate input and output buffers for each execution object
        std::vector<void *> buffers;
        for (auto &eo : execution_objects)
        {
            ArgInfo in  = { ArgInfo(malloc(frame_sz), frame_sz)};
            ArgInfo out = { ArgInfo(malloc(frame_sz), frame_sz)};
            eo->SetInputOutputBuffer(in, out);

            buffers.push_back(in.ptr());
            buffers.push_back(out.ptr());
        }

Step 5
======
Run the network on each input frame.  The frames are processed with available execution objects in a pipelined manner with additional num_eos iterations to flush the pipeline (epilogue).

.. code-block:: c++

        for (int frame_idx = 0; frame_idx < configuration.numFrames + num_eos; frame_idx++)
        {
            ExecutionObject* eo = execution_objects[frame_idx % num_eos].get();

            // Wait for previous frame on the same eo to finish processing
            if (eo->ProcessFrameWait())
                WriteFrame(*eo, output_data_file);

            // Read a frame and start processing it with current eo
            if (ReadFrame(*eo, frame_idx, configuration, input_data_file))
                eo->ProcessFrameStartAsync();
        }

For a complete example of using the API, refer any of the examples available at ``/usr/share/ti/tidl/examples`` on the EVM file system.

.. _Processor SDK Linux Software Developer's Guide: http://software-dl.ti.com/processor-sdk-linux/esd/docs/latest/linux/index.html
