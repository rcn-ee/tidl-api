.. _api-documentation:

*************
API Reference
*************

.. _api-ref-configuration:

Configuration
+++++++++++++
.. doxygenclass:: tidl::Configuration
    :members:

Configuration file
==================

TIDL API allows the user to create a Configuration object by reading from a file or by initializing it directly. Configuration settings supported by ``Configuration::ReadFromFile``:

    * numFrames
    * inWidth
    * inHeight
    * inNumChannels
    * preProcType
    * layerIndex2LayerGroupId

    * inData
    * outData

    * netBinFile
    * paramsBinFile

    * enableTrace

An example configuration file:

.. literalinclude:: ../../examples/layer_output/j11_v2_trace.txt
    :language: bash


.. _layer-group-override:

Overriding layer group assignment
=================================
The `TIDL device translation tool`_ assigns layer group ids to layers during the translation process. TIDL API 1.1 and higher allows the user to override this assignment by specifying explicit mappings. There are two ways for the user to provide an updated mapping:

1. Specify a mapping in the configuration file to indicate that layers 12, 13 and 14 are assigned to layer group 2:

.. code-block:: c++

    layerIndex2LayerGroupId = { {12, 2}, {13, 2}, {14, 2} }


2. User can also provide the layer index to group mapping in the code:

.. code-block:: c++

    Configuration c;
    c.ReadFromFile("test.cfg");
    c.layerIndex2LayerGroupId = { {12, 2}, {13, 2}, {14, 2} };


.. role:: cpp(code)
   :language: c++


.. _api-ref-executor:

Executor
++++++++
.. doxygenclass:: tidl::Executor
    :members:

.. _api-ref-eo:

ExecutionObject
+++++++++++++++
.. doxygenclass:: tidl::ExecutionObject
    :members:

.. _api-ref-eop:

ExecutionObjectPipeline
+++++++++++++++++++++++
.. doxygenclass:: tidl::ExecutionObjectPipeline
    :members:


.. refer https://breathe.readthedocs.io/en/latest/directives.html

.. _TIDL device translation tool: http://software-dl.ti.com/processor-sdk-linux/esd/docs/latest/linux/Foundational_Components_TIDL.html#import-process
