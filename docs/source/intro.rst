************
Introduction
************

TI Deep Learning (TIDL) API brings Deep Learning to the edge and enables applications to leverage TI's proprietary CNN/DNN implementation on Deep Learning Accelerators (DLAs) and C66x DSPs. TIDL will initially target Vision/2D use cases.

TIDL leverages the following techniques to optimize performance of CNN/DNN on TI's AM57x SoCs. For details on these techniques, refer [TODO: add link]

* Complexity reductions
* Sparse convolutions
* Quantization to fixed point (8-bit and 16-bit)

TIDL consists of the following components:

* A C++ API on ARM/Linux
* Translator tool to convert TensorFlow/Caffe networks to TIDL network format
* A graphviz based visualizer for TIDL network graphs

.. note:: Only certain Caffe/Tensor flow models can be translated to TIDL. There are constraints on supported layers that must be met. This is not a universal translator. 

Key Features
------------
Ease of use
+++++++++++
* Easily integrate TIDL APIs into other frameworks such as OpenCV
* Provides a common host abstraction for user applications across multiple compute engines (DLAs and C66x DSPs)

Low overhead
+++++++++++++
The execution time of TIDL APIs on the host is a fairly small percentage of the overall per-frame exection time. For example, with jseg21 network, 1024x512x3 frame size,  the APIs account for ~1.5% of overall per frame processing time of 320ms.

Software Architecture          
---------------------
The API consists of 3 classes with simple user interfaces:

* Configuration
* Executor
* ExecutionObject

Figure X shows the TIDL API software architecture.

.. image:: images/tidl-api.png
    :align: center


[TODO: Add text]

OpenCL v1.2 added support for custom devices. The OpenCL runtime for a custom device implements the standard OpenCL host API functions. However, a custom device does not support OpenCL-C programs. Host programs can invoke a fixed set of kernels built into the runtime. The DLAs on AM57x SoCs are modeled as OpenCL custom devices with a fixed set of built-in kernels.

Supported Layers
----------------

* Convolution
* Pooling (Average, Max)
* ReLU (including PReLU and ReLU6)
* ElementWise  (Add, Max, Product)
* Inner Product  (Fully Connected)
* SoftMax 
* Bias 
* Deconvolution 
* Concatenate 
* ArgMax 
* Scale 
* Batch Normalization 
* Crop 
* Slice 
* Flatten 
* Split 

.. note:: There are constraints on usage of these layers. See documentation for details. [TODO: add link]

Development Flow
----------------

.. image:: images/tidl-development-flow.png
    :align: center

