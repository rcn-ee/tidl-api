*******************************
TI Deep Learning (TIDL) example
*******************************

This example illustrates using the TI Neural Network (TINN) API to offload deep learning network processing from a Linux application to the C66x DSPs or DLAs on AM57x devices.

OpenCL v1.2 added support for custom devices. The OpenCL runtime for a custom device implements the standard OpenCL host API functions. However, a custom device does not support OpenCL-C programs. Host programs can invoke a fixed set of kernels built into the runtime. The DLAs on AM57x SoCs are modeled as OpenCL custom devices with a fixed set of built-in kernels corresponding to TIDL.

The TINN API is a C++ API to abstract lower level OpenCL host APIs for custom devices. The TINN API enables AM57x applications to leverage DLAs or DSPs for deep learning. The API:

* Enables easy integration of  TIDL into other frameworks such as OpenCV
* Is low overhead - OpenCL APIs account for ~1.5% of overall per frame processing time (224x224 frame with 3 channels) 
* Provides an example of using the OpenCL DLAs custom device TIDL kernels
* Provides a common abstraction for running TIDL networks on DLAs or C66x DSPs

.. note::
    DLA: TI Deep Learning Accelerator, also known as EVE.

TINN API
--------

The figure below describes the relationship between TINN APIs, the user's application and OpenCL host APIs.

.. figure:: ../images/tinn_api.png

The API consistes of 3 classes with simple user interfaces:

* Configuration
* Executor
* ExecutionObject

Using the TINN API
++++++++++++++++++

Step 1
======

Determine if there are any TIDL capable devices on the AM57x SoC:

.. code-block:: c++

    uint32_t num_dla = Executor::GetNumDevicesSupportingTIDL(DeviceType::DLA);
    uint32_t num_dsp = Executor::GetNumDevicesSupportingTIDL(DeviceType::DSP);

Step 2
======
Create a Configuration object by reading it from a file or by initializing it directly. The example below parses a configuration file and initializes the Configuration object. See ``tidl/testvecs/config/infer`` for examples of configuration files.

.. code::

    Configuration configuration;
    bool status = configuration.ReadFromFile(config_file);

.. note::
    Refer TIDL Translation Tool documentation for creating TIDL network and parameter binary files from TensorFlow and Caffe.

Step 3
======
Create an Executor with the approriate device type, set of devices and a configuration. In the snippet below, an Executor is created on 2 DLAs.

.. code-block:: c++

        DeviceIds ids = {DeviceId::ID0, DeviceId::ID1};
        Executor executor(DeviceType::DLA, ids, configuration);

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



Putting it together
===================
The code snippet :ref:`tidl_main` illustrates using the API to offload a network.

.. literalinclude:: ../../../../examples/tidl/main.cpp
    :name: tidl_main
    :caption: /usr/share/ti/examples/opencl/tidl/main.cpp
    :lines: 160-193,211-216,218-223
    :linenos:

For a complete example of using the API, refer ``/usr/share/ti/examples/opencl/tidl/main.cpp`` on the EVM filesystem.

TINN API documentation
----------------------

.. doxygennamespace:: tidl
    :project: TIDL
    :members:


