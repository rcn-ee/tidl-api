********
Examples
********

+---------------------+----------------------------------------------------------------+
| Example             | Description                                                    |
+---------------------+----------------------------------------------------------------+
| one_eo_per_frame    | Simple example to illustrate processing a single               |
|                     | frame with one :term:`EO` using the j11_v2 network.            |
|                     | Per-frame processing time for this network is farily similar   |
|                     | across EVE and C66x DSP. The enables frame processing to be    |
|                     | parallelized by distributing frames across all available EVE   |
|                     | and C66x cores.                                                |
+---------------------+----------------------------------------------------------------+
| two_eo_per_frame    | Simple example to illustrate processing a single               |
|                     | frame with two :term:`EOs<EO>` using the j11_v2 network.       |
+---------------------+----------------------------------------------------------------+
| imagenet            | Classification                                                 |
+---------------------+----------------------------------------------------------------+
| segmentation        | Pixel level segmentation                                       |
+---------------------+----------------------------------------------------------------+
| ssd_multibox        | Object detection                                               |
+---------------------+----------------------------------------------------------------+
| tidl_classification | Classification                                                 |
+---------------------+----------------------------------------------------------------+
| layer_output        | Illustrates using TIDL APIs to access output buffers           |
|                     | of intermediate :term:`Layer`s in the network.                 |
+---------------------+----------------------------------------------------------------+
| test                | Unit test. Tests supported networks on C66x and EVE            |
+---------------------+----------------------------------------------------------------+

The examples included in the tidl-api package demonstrate three categories of
deep learning networks: classification, segmentation and object detection.
``imagenet`` and ``segmentation`` can run on AM57x processors with either EVE or C66x cores.
``ssd_multibox`` requires AM57x processors with both EVE and C66x.  The performance
numbers that we present here were obtained on an AM5729 EVM, which
includes 2 Arm Cortex-A15 cores running at 1.5GHz, 2 EVE cores at 650MHz, and
2 DSP cores at 750MHz.

For each example, we report device processing time, host processing time,
and TIDL API overhead.  **Device processing time** is measured on the device,
from the moment processing starts for a frame till processing finishes.
**Host processing time** is measured on the host, from the moment
``ProcessFrameStartAsync()`` is called till ``ProcessFrameWait()`` returns
in user application.  It includes the TIDL API overhead, the OpenCL runtime
overhead, and the time to copy between user input/output data and
the padded TIDL internal buffers.

Imagenet
--------

The imagenet example takes an image as input and outputs 1000 probabilities.
Each probability corresponds to one object in the 1000 objects that the
network is pre-trained with.  Our example outputs top 5 predictions
as the most likely objects that the input image can be.

The following figure and tables shows an input image, top 5 predicted
objects as output, and the processing time on either EVE or DSP.

.. image:: ../../examples/test/testvecs/input/objects/cat-pet-animal-domestic-104827.jpeg
   :width: 600

.. table::

    ==== ==============
    Rank Object Classes
    ==== ==============
    1    tabby
    2    Egyptian_cat
    3    tiger_cat
    4    lynx
    5    Persian_cat
    ==== ==============

.. table::

   ====================== ==================== ============
   Device Processing Time Host Processing Time API Overhead
   ====================== ==================== ============
   EVE: 103.5 ms          104.8 ms             1.21 %
   **OR**
   DSP: 117.4 ms          118.4 ms             0.827 %
   ====================== ==================== ============

The particular network that we ran in this category, jacintonet11v2,
has 14 layers.  Input to the network is RGB image of 224x224.
User can specify whether to run the network on EVE or DSP
for acceleration.  We can see that EVE time is slightly faster than DSP time.
We can also see that the overall overhead is less than 1.3%.

.. note::
    The predicitions reported here are based on the output of the softmax
    layer in the network, which are not normalized to the real probabilities.

Segmentation
------------

The segmentation example takes an image as input and performs pixel-level
classification according to pre-trained categories.  The following figures
show a street scene as input and the scene overlaid with pixel-level
classifications as output: road in green, pedestrians in red, vehicles
in blue and background in gray.

.. image:: ../../examples/test/testvecs/input/roads/pexels-photo-972355.jpeg
   :width: 600

.. image:: images/pexels-photo-972355-seg.jpg
   :width: 600

The network we ran in this category is jsegnet21v2, which has 26 layers.
Input to the network is RGB image of size 1024x512.  The output is 1024x512
values, each value indicates which pre-trained category the current pixel
belongs to.  The example will take the network output, create an overlay,
and blend the overlay onto the original input image to create an output image.
From the reported time in the following table, we can see that this network
runs significantly faster on EVE than on DSP.  The API overhead is less than
1.1%.

.. table::

   ====================== ==================== ============
   Device Processing Time Host Processing Time API Overhead
   ====================== ==================== ============
   EVE: 248.7 ms          251.3 ms             1.02 %
   **OR**
   DSP: 813.2 ms          815.5 ms             0.281 %
   ====================== ==================== ============

.. _ssd-example:

SSD
---

SSD is the abbreviation for Single Shot multi-box Detector.
The ssd_multibox example takes an image as input and detects multiple
objects with bounding boxes according to pre-trained categories.
The following figures show another street scene as input and the scene
with recognized objects boxed as output: pedestrians in red,
vehicles in blue and road signs in yellow.

.. image:: ../../examples/test/testvecs/input/roads/pexels-photo-378570.jpeg
   :width: 600

.. image:: images/pexels-photo-378570-ssd.jpg
   :width: 600

The network we ran in this category is jdenet_ssd, which has 43 layers.
Input to the network is RGB image of size 768x320.  Output is a list of
boxes (up to 20), each box has information about the box coordinates, and
which pre-trained category that the object inside the box belongs to.
The example will take the network output, draw boxes accordingly,
and create an output image.
The network can be run entirely on either EVE or DSP.  But the best
performance comes with running the first 30 layers as a group on EVE
and the next 13 layers as another group on DSP.
Note the **AND** in the following table for the reported time.
The overall API overhead is about 1.61%.
Our end-to-end example shows how easy it is to assign a layers group id
to an *Executor* and how easy it is to construct an *ExecutionObjectPipeline*
to connect the output of one *Executor*'s *ExecutionObject*
to the input of another *Executor*'s *ExecutionObject*.

.. table::

   ====================== ==================== ============
   Device Processing Time Host Processing Time API Overhead
   ====================== ==================== ============
   EVE: 148.0 ms          150.1 ms             1.33 %
   **AND**
   DSP: 22.27 ms          23.06 ms             3.44 %
   **TOTAL**
   EVE+DSP: 170.3 ms      173.1 ms             1.61 %
   ====================== ==================== ============

Test
----
This example is used to test pre-converted networks included in the TIDL API package (``test/testvecs/config/tidl_models``). When run without any arguments, the program ``test_tidl`` will run all available networks on the C66x DSPs and EVEs available on the SoC. Use the ``-c`` option to specify a single network. Run ``test_tidl -h``  for details.

Running Examples
----------------

The examples are located in ``/usr/share/ti/tidl/examples`` on
the EVM file system.  Each example needs to be run its own directory.
Running an example with ``-h`` will show help message with option set.
The following code section shows how to run the examples, and
the test program that tests all supported TIDL network configs.

.. code-block:: shell

   root@am57xx-evm:~# cd /usr/share/ti/tidl-api/examples/imagenet/
   root@am57xx-evm:/usr/share/ti/tidl-api/examples/imagenet# make -j4
   root@am57xx-evm:/usr/share/ti/tidl-api/examples/imagenet# ./imagenet -t d
   Input: ../test/testvecs/input/objects/cat-pet-animal-domestic-104827.jpeg
   frame[0]: Time on device:  117.9ms, host:  119.3ms API overhead:   1.17 %
   1: tabby, prob = 0.996
   2: Egyptian_cat, prob = 0.977
   3: tiger_cat, prob = 0.973
   4: lynx, prob = 0.941
   5: Persian_cat, prob = 0.922
   imagenet PASSED

   root@am57xx-evm:/usr/share/ti/tidl-api/examples/imagenet# cd ../segmentation/; make -j4
   root@am57xx-evm:/usr/share/ti/tidl-api/examples/segmentation# ./segmentation -i ../test/testvecs/input/roads/pexels-photo-972355.jpeg
   Input: ../test/testvecs/input/roads/pexels-photo-972355.jpeg
   frame[0]: Time on device:  296.5ms, host:  303.2ms API overhead:   2.21 %
   Saving frame 0 overlayed with segmentation to: overlay_0.png
   segmentation PASSED

   root@am57xx-evm:/usr/share/ti/tidl-api/examples/segmentation# cd ../ssd_multibox/; make -j4
   root@am57xx-evm:/usr/share/ti/tidl-api/examples/ssd_multibox# ./ssd_multibox -i ../test/testvecs/input/roads/pexels-photo-378570.jpeg
   Input: ../test/testvecs/input/roads/pexels-photo-378570.jpeg
   frame[0]: Time on EVE:  175.2ms, host:    179ms API overhead:    2.1 %
   frame[0]: Time on DSP:  21.06ms, host:  22.43ms API overhead:   6.08 %
   Saving frame 0 with SSD multiboxes to: multibox_0.png
   Loop total time (including read/write/print/etc):  423.8ms
   ssd_multibox PASSED

   root@am57xx-evm:/usr/share/ti/tidl-api/examples/ssd_multibox# cd ../test; make -j4
   root@am57xx-evm:/usr/share/ti/tidl-api/examples/test# ./test_tidl
   API Version: 01.00.00.d91e442
   Running dense_1x1 on 2 devices, type EVE
   frame[0]: Time on device:  134.3ms, host:  135.6ms API overhead:  0.994 %
   dense_1x1 : PASSED
   Running j11_bn on 2 devices, type EVE
   frame[0]: Time on device:  176.2ms, host:  177.7ms API overhead:  0.835 %
   j11_bn : PASSED
   Running j11_cifar on 2 devices, type EVE
   frame[0]: Time on device:  53.86ms, host:  54.88ms API overhead:   1.85 %
   j11_cifar : PASSED
   Running j11_controlLayers on 2 devices, type EVE
   frame[0]: Time on device:  122.9ms, host:  123.9ms API overhead:  0.821 %
   j11_controlLayers : PASSED
   Running j11_prelu on 2 devices, type EVE
   frame[0]: Time on device:  300.8ms, host:  302.1ms API overhead:  0.437 %
   j11_prelu : PASSED
   Running j11_v2 on 2 devices, type EVE
   frame[0]: Time on device:  124.1ms, host:  125.6ms API overhead:   1.18 %
   j11_v2 : PASSED
   Running jseg21 on 2 devices, type EVE
   frame[0]: Time on device:    367ms, host:    374ms API overhead:   1.88 %
   jseg21 : PASSED
   Running jseg21_tiscapes on 2 devices, type EVE
   frame[0]: Time on device:  302.2ms, host:  308.5ms API overhead:   2.02 %
   frame[1]: Time on device:  301.9ms, host:  312.5ms API overhead:   3.38 %
   frame[2]: Time on device:  302.7ms, host:  305.9ms API overhead:   1.04 %
   frame[3]: Time on device:  301.9ms, host:    305ms API overhead:   1.01 %
   frame[4]: Time on device:  302.7ms, host:  305.9ms API overhead:   1.05 %
   frame[5]: Time on device:  301.9ms, host:  305.5ms API overhead:   1.17 %
   frame[6]: Time on device:  302.7ms, host:  305.9ms API overhead:   1.06 %
   frame[7]: Time on device:  301.9ms, host:    305ms API overhead:   1.02 %
   frame[8]: Time on device:    297ms, host:  300.3ms API overhead:   1.09 %
   Comparing frame: 0
   jseg21_tiscapes : PASSED
   Running smallRoi on 2 devices, type EVE
   frame[0]: Time on device:  2.548ms, host:  3.637ms API overhead:   29.9 %
   smallRoi : PASSED
   Running squeeze1_1 on 2 devices, type EVE
   frame[0]: Time on device:  292.9ms, host:  294.6ms API overhead:  0.552 %
   squeeze1_1 : PASSED

   Multiple Executor...
   Running network tidl_config_j11_v2.txt on EVEs: 1  in thread 0
   Running network tidl_config_j11_cifar.txt on EVEs: 0  in thread 1
   Multiple executors: PASSED
   Running j11_bn on 2 devices, type DSP
   frame[0]: Time on device:  170.5ms, host:  171.5ms API overhead:  0.568 %
   j11_bn : PASSED
   Running j11_controlLayers on 2 devices, type DSP
   frame[0]: Time on device:  416.4ms, host:  417.1ms API overhead:  0.176 %
   j11_controlLayers : PASSED
   Running j11_v2 on 2 devices, type DSP
   frame[0]: Time on device:    118ms, host:  119.2ms API overhead:   1.01 %
   j11_v2 : PASSED
   Running jseg21 on 2 devices, type DSP
   frame[0]: Time on device:   1123ms, host:   1128ms API overhead:  0.443 %
   jseg21 : PASSED
   Running jseg21_tiscapes on 2 devices, type DSP
   frame[0]: Time on device:  812.3ms, host:  817.3ms API overhead:  0.614 %
   frame[1]: Time on device:  812.6ms, host:  818.6ms API overhead:  0.738 %
   frame[2]: Time on device:  812.3ms, host:  815.1ms API overhead:  0.343 %
   frame[3]: Time on device:  812.7ms, host:  815.2ms API overhead:  0.312 %
   frame[4]: Time on device:  812.3ms, host:  815.1ms API overhead:  0.353 %
   frame[5]: Time on device:  812.6ms, host:  815.1ms API overhead:  0.302 %
   frame[6]: Time on device:  812.2ms, host:  815.1ms API overhead:  0.357 %
   frame[7]: Time on device:  812.6ms, host:  815.2ms API overhead:  0.315 %
   frame[8]: Time on device:    812ms, host:    815ms API overhead:  0.367 %
   Comparing frame: 0
   jseg21_tiscapes : PASSED
   Running smallRoi on 2 devices, type DSP
   frame[0]: Time on device:  14.21ms, host:  14.94ms API overhead:   4.89 %
   smallRoi : PASSED
   Running squeeze1_1 on 2 devices, type DSP
   frame[0]: Time on device:    960ms, host:  961.1ms API overhead:  0.116 %
   squeeze1_1 : PASSED
   tidl PASSED

Image input
^^^^^^^^^^^

The image input option, ``-i <image>``, takes an image file as input.
You can supply an image file with format that OpenCV can read, since
we use OpenCV for image pre/post-processing.  When ``-f <number>`` option
is used, the same image will be processed repeatedly.

Camera (live video) input
^^^^^^^^^^^^^^^^^^^^^^^^^

The input option, ``-i camera<number>``, enables live frame inputs
from camera.  ``<number>`` is the video input port number
of your camera in Linux.  Use the following command to check video
input ports.  The number defaults to ``1`` for TMDSCM572X camera module
used on AM57x EVMs.  You can use ``-f <number>`` to specify the number
of frames you want to process.

.. code-block:: shell

  root@am57xx-evm:~# v4l2-ctl --list-devices
  omapwb-cap (platform:omapwb-cap):
        /dev/video11

  omapwb-m2m (platform:omapwb-m2m):
        /dev/video10

  vip (platform:vip):
        /dev/video1

  vpe (platform:vpe):
        /dev/video0


Pre-recorded video (mp4/mov/avi) input
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The input option, ``-i <name>.{mp4,mov,avi}``, enables frame inputs from
pre-recorded video file in mp4, mov or avi format.  If you have a video in
a different OpenCV-supported format/suffix, you can simply create a softlink
with one of the mp4, mov or avi suffixes and feed it into the example.
Again, use ``-f <number>`` to specify the number of frames you want to process.

Displaying video output
^^^^^^^^^^^^^^^^^^^^^^^

When using video input, live or pre-recorded, the example will display
the output in a window using OpenCV.  If you have a LCD screen attached
to the EVM, you will need to kill the ``matrix-gui`` first in order to
see the example display window, as shown in the following example.

.. code-block:: shell

  root@am57xx-evm:/usr/share/ti/tidl/examples/ssd_multibox# /etc/init.d/matrix-gui-2.0 stop
  Stopping Matrix GUI application.
  root@am57xx-evm:/usr/share/ti/tidl/examples/ssd_multibox# ./ssd_multibox -i camera -f 100
  Input: camera
  init done
  Using Wayland-EGL
  wlpvr: PVR Services Initialised
  Using the 'xdg-shell-v5' shell integration
  ... ...
  root@am57xx-evm:/usr/share/ti/tidl/examples/ssd_multibox# /etc/init.d/matrix-gui-2.0 start
  /usr/share/ti/tidl/examples/ssd_multibox
  Removing stale PID file /var/run/matrix-gui-2.0.pid.
  Starting Matrix GUI application.
