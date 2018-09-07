************
Introduction
************

TI Deep Learning (TIDL) API brings deep learning to the edge by enabling applications to leverage TI's proprietary, highly optimized CNN/DNN implementation on the EVE and C66x DSP compute engines. TIDL will initially target Vision/2D use cases on AM57x Sitara |(TM)| Processors.

The TIDL API leverages TI's `OpenCL`_ |(TM)| product to offload deep learning applications to both EVE(s) and DSP(s).  The TIDL API significantly improves the out-of-box deep learning experience for users and enables them to focus on their overall use case. They do not have to spend time on the mechanics of Arm |(R)| ↔ DSP/EVE communication or implementing optimized network layers on EVE(s) and/or DSP(s).  The API allows customers to easily integrate frameworks such as OpenCV and rapidly prototype deep learning applications.

.. note::

    This User's Guide focuses on the TIDL API. For information on TIDL such as the overall development flow, techniques to optimize performance of CNN/DNN on TI's processors, performance/benchmarking data and list of supported layers, see the TIDL section in the `Processor SDK Linux Software Developer's Guide (TIDL chapter)`_.

Key Features
------------
**Ease of use**

* Easily integrate TIDL APIs into other frameworks such as `OpenCV`_
* Provides simple host abstractions for user applications to run a network across multiple compute cores (EVEs and C66x DSPs). Refer :ref:`use-case-1` and :ref:`use-case-2` for details.

**Low overhead**

The execution time of TIDL APIs on the host is a fairly small percentage of the overall per-frame execution time. For example, with jseg21 network, 1024x512 frame with 3 channels, the APIs account for ~1.5% of overall per-frame processing time.

Development Flow
----------------

.. _`TIDL Development flow`:

.. figure:: images/tidl-development-flow.png
    :align: center
    :scale: 50

    Development flow with TIDL APIs

:numref:`TIDL Development flow` shows the overall development process. Deep learning consists to two stages: training at development stage and inference at deployment stage.  Training involves designing neural network model, running training data through the network to tune the model parameters.  Inference takes the pre-trained model including parameters, applies to new input and produces output.  Training is computationally intensive and is done using frameworks such as Caffe/TensorFlow. Once the network is trained, the TIDL converter tool can be used to translate the network and parameters to TIDL. The `Processor SDK Linux Software Developer's Guide (TIDL chapter)`_ provides details on the development flow and and the converter tool.

.. _Processor SDK Linux Software Developer's Guide: http://software-dl.ti.com/processor-sdk-linux/esd/docs/latest/linux/index.html
.. _Processor SDK Linux Software Developer's Guide (TIDL chapter): http://software-dl.ti.com/processor-sdk-linux/esd/docs/latest/linux/Foundational_Components_TIDL.html
.. _OpenCV: http://software-dl.ti.com/processor-sdk-linux/esd/docs/latest/linux/Foundational_Components.html#opencv
.. _OpenCL: http://software-dl.ti.com/mctools/esd/docs/opencl/index.html

.. |(TM)| unicode:: U+2122
    :ltrim:

.. |(R)| unicode:: U+00AE
    :ltrim:
