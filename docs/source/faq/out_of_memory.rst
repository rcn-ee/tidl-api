##################################################
Why do I get an assertion failure from malloc_ddr?
##################################################

Application execution fails with the following error message:

.. code:: shell

   tidl: device_alloc.h:31: T* tidl::malloc_ddr(size_t) [with T = char; size_t = unsigned int]: Assertion `val != nullptr' failed

Allocations not freed
+++++++++++++++++++++
One possible reason is that previous runs of the application were aborted (e.g. using :kbd:`Ctrl-C`) and did not release their allocations. Use the ``ti-mct-heap-check`` command with "-c" option to clean up.

.. code:: bash

   root@am57xx-evm:~# ti-mct-heap-check -c
   -- ddr_heap1 ------------------------------
      Addr : 0xa2000000
      Size : 0xa000000
      Avail: 0xa000000
      Align: 0x80
   -----------------------------------------

.. _opencl-global-memory:

Insufficient OpenCL global memory
+++++++++++++++++++++++++++++++++
Another possible reason is that total memory requirement specified in the ``Configuration`` using EXTMEM_HEAP_SIZE and PARAM_HEAP_SIZE exceeds default memory available for OpenCL.  Follow the instructions below to increase the amount of CMEM (contiguous memory available for OpenCL)

.. code:: bash

   $ sudo apt-get install device-tree-compiler # In case dtc is not already installed
   $ scp root@am57:/boot/am57xx-evm-reva3.dtb .
   $ dtc -I dtb -O dts am57xx-evm-reva3.dtb -o am57xx-evm-reva3.dts
   $ cp am57xx-evm-reva3.dts am57xx-evm-reva3.dts.orig
   $ # increase cmem block size
   $ diff -u am57xx-evm-reva3.dts.orig am57xx-evm-reva3.dts
   --- am57xx-evm-reva3.dts.orig    2018-01-11 14:47:51.491572739 -0600
   +++ am57xx-evm-reva3.dts    2018-01-16 15:43:33.981431971 -0600
   @@ -5657,7 +5657,7 @@
            };

            cmem_block_mem@a0000000 {
   -            reg = <0x0 0xa0000000 0x0 0xc000000>;
   +            reg = <0x0 0xa0000000 0x0 0x18000000>;
                no-map;
                status = "okay";
                linux,phandle = <0x13c>;
   @@ -5823,7 +5823,7 @@
            cmem_block@0 {
                reg = <0x0>;
                memory-region = <0x13c>;
   -            cmem-buf-pools = <0x1 0x0 0xc000000>;
   +            cmem-buf-pools = <0x1 0x0 0x18000000>;
            };

            cmem_block@1 {
   $ dtc -I dts -O dtb am57xx-evm-reva3.dts -o am57xx-evm-reva3.dtb
   $ scp am57xx-evm-reva3.dtb root@am57:/boot/
   # reboot to make memory changes effective (run "cat /proc/iomem" to check)

For further details, refer the OpenCL User's Guide, section :ref:`Changing DDR3 Partition for OpenCL <opencl:CHANGE_DDR3_PARTITION_FOR_OPENCL>`

