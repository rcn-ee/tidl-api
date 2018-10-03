#########
Changelog
#########

1.2.0 [Processor Linux SDK 5.2]
===============================
**Added**

* Python 3 bindings for TIDL API

**Removed**

* Configuration::enableInternalInput, not used.
* Execution::GetExecutionObjects. Use Execution::operator[] and Execution::GetNumExecutionObjects() instead. See :ref:`examples` for usage.

1.1.0 [Processor Linux SDK 5.1]
===============================
**Added**

* :term:`ExecutionObjectPipeline` class to hide complexity of executing network across C66x/EVE
* API methods for tracing outputs from intermediate network layers - see :ref:`network_layer_output`.
* Support for updating layer group id assignment before execution - see :ref:`layer-group-override`.
* Provide feedback to the user on parameter and network heap size requirements - see :ref:`sizing_device_heaps`.


1.0.0 [Processor Linux SDK 5.0]
===============================
First release of the TI Deep Learning API. TIDL API brings deep learning to the edge by enabling applications to leverage TI's proprietary, highly optimized CNN/DNN implementation on the EVE and C66x DSP compute engines. TIDL will initially target Vision/2D use cases.

**Supported AM57x Sitara Processors**

 * `AM571x`_ (offload to C66x DSPs)
 * `AM5728`_ (offload to C66x DSPs)
 * `AM574x`_ (offload to EVEs and C66x DSPs)

**Supported Evaluation Modules (EVMs)**

 * `AM572x EVM`_
 * `AM571x IDK EVM`_
 * `AM574x IDK EVM`_


.. _AM572x EVM:  http://www.ti.com/tool/tmdsevm572x
.. _AM571x IDK EVM:  http://www.ti.com/tool/tmdxidk5718
.. _AM574x IDK EVM:  http://www.ti.com/tool/tmdsidk574
.. _AM571x:     http://www.ti.com/processors/sitara/arm-cortex-a15/am57x/products.html#p2098=1%20C66x&p809=2;2
.. _AM5728:     http://www.ti.com/product/AM5728
.. _AM574x:     http://www.ti.com/processors/sitara/arm-cortex-a15/am57x/products.html#p2098=2%20C66x&p815=ECC
