#########
Changelog
#########

1.2.0 [Processor Linux SDK 5.2]
===============================
**Added**

#. Execution Graph generation

    Enable a two phase approach to generating execution graphs. Use the
    following API function to enable timestamp generation:

    .. code::

        bool EnableTimeStamps(const std::string& file = "timestamp.log", size_t num_frames=32);

    The generated log file can be viewed by using the execution_graph.py script. Refer to :ref:`execution-graph` for details.

#. Python 3 bindings for TIDL API

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
