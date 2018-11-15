.. _examples:

********
Examples
********

.. list-table:: TIDL API Examples
   :header-rows: 1
   :widths: 12 43 20 25

   * - Example
     - Description
     - Compute cores
     - Input image
   * - one_eo_per_frame
     - Processes a single frame with one :term:`EO` using the j11_v2 network. Throughput is increased by distributing frame processing across EOs. Refer :ref:`use-case-1`.
     - EVE or C66x
     - Pre-processed image read from file.
   * - two_eo_per_frame
     - Processes a single frame with an :term:`EOP` using the j11_v2 network to reduce per-frame processing latency. Also increases throughput by distributing frame processing across EOPs. The EOP consists of two EOs. Refer :ref:`use-case-2`.
     - EVE and C66x (network is split across both EVE and C66x)
     - Pre-processed image read from file.
   * - two_eo_per_frame_opt
     - Builds on ``two_eo_per_frame``. Adds double buffering to improve performance. Refer :ref:`use-case-3`.
     - EVE and C66x (network is split across both EVE and C66x)
     - Pre-processed image read from file.

   * - imagenet
     - Classification example
     - EVE or C66x
     - OpenCV used to read input image from file or capture from camera.
   * - segmentation
     - Pixel level segmentation example
     - EVE or C66x
     - OpenCV used to read input image from file or capture from camera.
   * - ssd_multibox
     - Object detection
     - EVE and C66x (network is split across both EVE and C66x)
     - OpenCV used to read input image from file or capture from camera.
   * - classification
     - Classification example, called from the Matrix GUI.
     - EVE or C66x
     - OpenCV used to read input image from file or capture from camera.
   * - mcbench
     - Used to benchmark supported networks. Refer ``mcbench/scripts`` for command line options.
     - EVE or C66x
     - Pre-processed image read from file.
   * - layer_output
     - Illustrates using TIDL APIs to access output buffers of intermediate :term:`layers<Layer>` in the network.
     - EVE or C66x
     - Pre-processed image read from file.
   * - test
     - This example is used to test pre-converted networks included in the TIDL API package (``test/testvecs/config/tidl_models``). When run without any arguments, the program ``test_tidl`` will run all available networks on the C66x DSPs and EVEs available on the SoC. Use the ``-c`` option to specify a single network. Run ``test_tidl -h``  for details.
     - C66x and EVEs (if available)
     - Pre-processed image read from file.

The included examples demonstrate three categories of deep learning networks: classification, segmentation and object detection.  ``imagenet`` and ``segmentation`` can run on AM57x processors with either EVE or C66x cores.  ``ssd_multibox`` requires AM57x processors with both EVE and C66x. The examples are available at ``/usr/share/ti/tidl/examples`` on the EVM file system and in the linux devkit.

The performance numbers were obtained using:

* `AM574x IDK EVM`_ with the Sitara `AM5749`_ Processor - 2 Arm Cortex-A15 cores running at 1.0GHz, 2 EVE cores at 650MHz, and 2 C66x cores at 750MHz.
* `Processor SDK Linux`_ v5.1 with TIDL API v1.1

For each example, device processing time, host processing time,
and TIDL API overhead is reported.

* **Device processing time** is measured on the device, from the moment processing starts for a frame till processing finishes.
* **Host processing time** is measured on the host, from the moment ``ProcessFrameStartAsync()`` is called till ``ProcessFrameWait()`` returns in user application.  It includes the TIDL API overhead, the OpenCL runtime overhead, and the time to copy user input data into padded TIDL internal buffers. ``Host processing time = Device processing time + TIDL API overhead``.


Imagenet
--------

The imagenet example takes an image as input and outputs 1000 probabilities.
Each probability corresponds to one object in the 1000 objects that the
network is pre-trained with.  The example outputs top 5 predictions for a given input image.

The following figure and tables shows an input image, top 5 predicted
objects as output, and the processing time on either EVE or C66x.

.. image:: ../../examples/test/testvecs/input/objects/cat-pet-animal-domestic-104827.jpeg
   :width: 600


==== ==============
Rank Object Classes
==== ==============
1    tabby
2    Egyptian_cat
3    tiger_cat
4    lynx
5    Persian_cat
==== ==============

=======   ====================== ==================== ============
Device    Device Processing Time Host Processing Time API Overhead
=======   ====================== ==================== ============
EVE       106.5 ms               107.9 ms             1.37 %
C66x      117.9 ms               118.7 ms             0.93 %
=======   ====================== ==================== ============

The :term:`network<Network>` used in the example is jacintonet11v2. It has
14 layers. Input to the network is RGB image of 224x224. Users can specify whether to run the network on EVE or C66x.

The example code sets ``buffer_factor`` to 2 to create duplicated
ExecutionObjectPipelines with identical ExecutionObjects to
perform double buffering, so that host pre/post-processing can be overlapped
with device processing (see comments in the code for details).
The following table shows the loop overall time over 10 frames
with single buffering and double buffering,
``./imagenet -f 10 -d <num> -e <num>``.

.. list-table:: Loop overall time over 10 frames
   :header-rows: 1

   * - Device(s)
     - Single Buffering (buffer_factor=1)
     - Double Buffering (buffer_factor=2)
   * - 1 EVE
     - 1744 ms
     - 1167 ms
   * - 2 EVEs
     - 966 ms
     - 795 ms
   * - 1 C66x
     - 1879 ms
     - 1281 ms
   * - 2 C66xs
     - 1021 ms
     - 814 ms

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

The :term:`network<Network>` used in the example is jsegnet21v2. It has
26 layers.  Users can specify whether to run the network on EVE or C66x.
Input to the network is RGB image of size 1024x512.  The output is 1024x512
values, each value indicates which pre-trained category the current pixel
belongs to.  The example will take the network output, create an overlay,
and blend the overlay onto the original input image to create an output image.
From the reported time in the following table, we can see that this network
runs significantly faster on EVE than on C66x.

=======     ====================== ==================== ============
Device      Device Processing Time Host Processing Time API Overhead
=======     ====================== ==================== ============
EVE         251.8 ms               254.2 ms             0.96 %
C66x        812.7 ms               815.0 ms             0.27 %
=======     ====================== ==================== ============

The example code sets ``buffer_factor`` to 2 to create duplicated
ExecutionObjectPipelines with identical ExecutionObjects to
perform double buffering, so that host pre/post-processing can be overlapped
with device processing (see comments in the code for details).
The following table shows the loop overall time over 10 frames
with single buffering and double buffering,
``./segmentation -f 10 -d <num> -e <num>``.

.. list-table:: Loop overall time over 10 frames
   :header-rows: 1

   * - Device(s)
     - Single Buffering (buffer_factor=1)
     - Double Buffering (buffer_factor=2)
   * - 1 EVE
     - 5233 ms
     - 3017 ms
   * - 2 EVEs
     - 3032 ms
     - 3015 ms
   * - 1 C66x
     - 10890 ms
     - 8416 ms
   * - 2 C66xs
     - 5742 ms
     - 4638 ms

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
The network can be run entirely on either EVE or C66x.  However, the best
performance comes with running the first 30 layers as a group on EVE
and the next 13 layers as another group on C66x.
Our end-to-end example shows how easy it is to assign a :term:`Layer Group` id
to an :term:`Executor` and how easy it is to construct an :term:`ExecutionObjectPipeline` to connect the output of one *Executor*'s :term:`ExecutionObject`
to the input of another *Executor*'s *ExecutionObject*.

========      ====================== ==================== ============
Device        Device Processing Time Host Processing Time API Overhead
========      ====================== ==================== ============
EVE+C66x      169.5ms                172.0ms              1.68 %
========      ====================== ==================== ============

The example code sets ``pipeline_depth`` to 2 to create duplicated
ExecutionObjectPipelines with identical ExecutionObjects to
perform pipelined execution at the ExecutionObject level.
The side effect is that it also overlaps host pre/post-processing
with device processing (see comments in the code for details).
The following table shows the loop overall time over 10 frames
with pipelining at ExecutionObjectPipeline level
versus ExecutionObject level.
``./ssd_multibox -f 10 -d <num> -e <num>``.

.. list-table:: Loop overall time over 10 frames
   :header-rows: 1

   * - Device(s)
     - pipeline_depth=1
     - pipeline_depth=2
   * - 1 EVE + 1 C66x
     - 2900 ms
     - 1735 ms
   * - 2 EVEs + 2 C66xs
     - 1630 ms
     - 1408 ms

Running Examples
----------------

The examples are located in ``/usr/share/ti/tidl/examples`` on
the EVM file system.  **Each example needs to be run in its own directory** due to relative paths to configuration files.
Running an example with ``-h`` will show help message with option set.
The following listing illustrates how to build and run the examples.

.. code-block:: shell

   root@am57xx-evm:~/tidl-api/examples/imagenet# ./imagenet
   Input: ../test/testvecs/input/objects/cat-pet-animal-domestic-104827.jpeg
   frame[  0]: Time on EVE0: 106.50 ms, host: 107.96 ms API overhead: 1.35 %
   1: tabby
   2: Egyptian_cat
   3: tiger_cat
   4: lynx
   5: Persian_cat
   Loop total time (including read/write/opencv/print/etc):  202.6ms
   imagenet PASSED

   root@am57xx-evm:~/tidl-api/examples/segmentation# ./segmentation
   Input: ../test/testvecs/input/000100_1024x512_bgr.y
   frame[  0]: Time on EVE0: 251.74 ms, host: 258.02 ms API overhead: 2.43 %
   Saving frame 0 to: frame_0.png
   Saving frame 0 overlayed with segmentation to: overlay_0.png
   frame[  1]: Time on EVE0: 251.76 ms, host: 255.79 ms API overhead: 1.58 %
   Saving frame 1 to: frame_1.png
   Saving frame 1 overlayed with segmentation to: overlay_1.png
   ...
   frame[  8]: Time on EVE0: 251.75 ms, host: 254.21 ms API overhead: 0.97 %
   Saving frame 8 to: frame_8.png
   Saving frame 8 overlayed with segmentation to: overlay_8.png
   Loop total time (including read/write/opencv/print/etc):   4809ms
   segmentation PASSED

   root@am57xx-evm:~/tidl-api/examples/ssd_multibox# ./ssd_multibox
   Input: ../test/testvecs/input/preproc_0_768x320.y
   frame[  0]: Time on EVE0+DSP0: 169.44 ms, host: 173.56 ms API overhead: 2.37 %
   Saving frame 0 to: frame_0.png
   Saving frame 0 with SSD multiboxes to: multibox_0.png
   Loop total time (including read/write/opencv/print/etc):  320.2ms
   ssd_multibox PASSED


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


.. _AM574x IDK EVM:  http://www.ti.com/tool/tmdsidk574
.. _AM5749: http://www.ti.com/product/AM5749/
.. _Processor SDK Linux: http://software-dl.ti.com/processor-sdk-linux/esd/AM57X/latest/index_FDS.html
