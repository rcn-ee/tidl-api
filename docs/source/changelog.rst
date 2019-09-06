#########
Changelog
#########

1.3.1 [Processor Linux SDK 6.1]
===============================
**Added**

#. Added imagenet python example.

**Changed**

#. Trace data now also include dataQ/minValue/maxValue info.

#. ssd_multibox example now can also run the whole netework on a single core.

#. Shipped network binaries are in new file format, of size 484384 bytes.

1.3.0 [Processor Linux SDK 5.3]
===============================
**Added**

#. Added DSP support for MNIST example.

**Changed**

#. PSDK 5.3 and TIDL-API 1.3 use a slightly modified TIDL network binary
   format to support ONNX network import.  TIDL-API 1.3, as well as
   tidl-viewer utility, can read both network formats before and after the
   change.  Network binary in TIDL-API 1.2 and earlier is of size 483364
   bytes, network binary in TIDL-API 1.3 is of size 484384 bytes.

#. Improved performance of concat layer on C66x DSP.

1.2.2 [Processor Linux SDK 5.2]
===============================
**Added**

#. Updated API implementation to minimize TIDL API/OpenCL dispatch overhead using multiple execution contexts in the :term:`ExecutionObject`.
   Refer to :ref:`mnist-example` example for details.

#. Execution Graph generation

    Enable a two phase approach to generating execution graphs. Use the
    following API function to enable timestamp generation:

    .. code:: cpp

        bool EnableTimeStamps(const std::string& file = "timestamp.log", size_t num_frames=32);

    The generated log file can be viewed by using the execution_graph.py script. Refer to :ref:`execution-graph` for details.

#. Added Python 3 bindings for TIDL API. See the ``examples/pybind`` directory for examples of using the Python bindings. Set PYTHONPATH to the location of ``tidl.so``.

   .. code:: bash

        root@am57xx-evm:~# export PYTHONPATH=/home/root/tidl-api/tidl_api
        root@am57xx-evm:~# python3
        >>> import tidl
        >>> help (tidl)

**Removed**

#. Configuration::enableInternalInput. Not used by the API.

#. Execution::GetExecutionObjects().

   Use Execution::operator[] and Execution::GetNumExecutionObjects() instead.
   See :ref:`examples` for usage.

#. The timing methods for host execution in EOPs and EOs:

   * GetProcessTimeInMilliSeconds()
   * GetHostProcessTimeInMilliSeconds()

   These methods were replaced by a timestamp based approach because they were
   no longer accurate with multiple ExecutionObject contexts and pipelining.
   Refer to :ref:`execution-graph` for details.

1.1.0 [Processor Linux SDK 5.1]
===============================
**Added**

#. :term:`ExecutionObjectPipeline` class to hide complexity of executing network across C66x/EVE
#. API methods for tracing outputs from intermediate network layers - see :ref:`network_layer_output`.
#. Support for updating layer group id assignment before execution - see :ref:`layer-group-override`.
#. Provide feedback to the user on parameter and network heap size requirements - see :ref:`sizing_device_heaps`.


1.0.0 [Processor Linux SDK 5.0]
===============================
First release of the TI Deep Learning API. TIDL API brings deep learning to the edge by enabling applications to leverage TI's proprietary, highly optimized CNN/DNN implementation on the EVE and C66x DSP compute engines. TIDL will initially target Vision/2D use cases.

**Supported AM57x Sitara Processors**

 * `AM5749`_ (offload to EVEs and C66x DSPs)
 * `AM571x`_ (offload to C66x DSPs)
 * `AM5728`_ (offload to C66x DSPs)
 * `AM5748`_ (offload to C66x DSPs)

**Supported Evaluation Modules (EVMs)**

 * `AM574x IDK EVM`_
 * `AM572x EVM`_
 * `AM571x IDK EVM`_


.. _AM572x EVM:  http://www.ti.com/tool/tmdsevm572x
.. _AM571x IDK EVM:  http://www.ti.com/tool/tmdxidk5718
.. _AM574x IDK EVM:  http://www.ti.com/tool/tmdsidk574
.. _AM571x:     http://www.ti.com/processors/sitara/arm-cortex-a15/am57x/products.html#p2098=1%20C66x&p809=2;2
.. _AM5728:     http://www.ti.com/product/AM5728
.. _AM5748:     http://www.ti.com/product/am5748
.. _AM5749:     http://www.ti.com/product/am5749
.. _AM574x:     http://www.ti.com/processors/sitara/arm-cortex-a15/am57x/products.html#p2098=2%20C66x&p815=ECC
