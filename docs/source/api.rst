.. _api-documentation:

******************
TIDL API Reference
******************

Configuration
-------------
The ``Configuration`` object is used to specify various parameters required for network execution. Applications can directly initialize fields in an instance of ``Configuration`` or use the ``ReadFromFile`` method to read the configuration from a file. The following sections describe the commonly used fields available in the ``Configuration`` class:

Input image description
+++++++++++++++++++++++
.. data::  std::string Configuration.inData

    Path to the input image file. This field is not used by the TIDL API itself. It can be used by applications to load an input image into a buffer. Can be empty if the application uses frameworks such as OpenCV to read images. See ``test/main.cpp`` for example usage.

.. data:: std::size_t Configuration.inHeight

    Height of the input image. Used by the API, must be specified.

.. data:: std::size_t Configuration.inWidth

    Width of the input image. Used by the API, must be specified.

.. data:: std::size_t Configuration.inNumChannels

    Number of channels in the input image. Used by the API, must be specified.

Output description
++++++++++++++++++

.. data::  std::string Configuration.outData

    Path to the output image file. This field is not used by the TIDL API itself. It can be used by applications to write a buffer to file. Can be empty if the application uses frameworks such as OpenCV to read images. See ``test/main.cpp`` for example usage.

Network
+++++++

.. data:: std::string Configuration.netBinFile

    Path to the TIDL network binary file. Used by the API, must be specified.

.. data:: std::string Configuration.paramBinFile

    Path to the TIDL parameter file. Used by the API, must be specified.

.. data:: std::map<int, int> layerIndex2LayerGroupId

    Map of layer index to layer group id. Used to override layer group assigment for layers. Any layer not specified in this map will retain its existing mapping.

Memory Management
+++++++++++++++++
The ``Configuration`` object specifies the sizes of 2 heaps. These heaps are allocated from OpenCL global memory that is shared across the host and device. Refer section :ref:`opencl-global-memory` for steps to increase the size of the OpenCL global memory heap.

.. data:: std::size_t Configuration.PARAM_HEAP_SIZE

    This field is used to specify the size of the device heap used for network parameters. The size depends on the size of the parameter binary file. For example, ``jsegnet21v2``'s parameter file, ``tidl_param_jsegnet21v2.bin`` is 2.6MB. Due to alignment reasons, the parameter heap must be 10% larger than the binary file size - in this case, 2.9MB. The constructor for ``Configuration`` sets PARAM_HEAP_SIZE to 9MB. There is one parameter heap for each instance of ``Executor`` .

.. data:: std::size_t Configuration.EXTMEM_HEAP_SIZE

    This field is used to specify the size of the device heap used for all allocations other than network parameters. The constructor for ``Configuration`` sets EXTMEM_HEAP_SIZE to 64MB.  There is one external memory heap for each instance of ``ExecutionObject``

Debug
+++++
.. data:: bool enableOutputTrace;

    Enable tracing of output buffers associated with each layer.



API Reference
-------------

.. doxygennamespace:: tidl
    :project: TIDL
    :members:


