TI Deep Learning (TIDL) API
---------------------------

TIDL API brings Deep Learning to the edge and enables Linux applications to leverage TI’s proprietary CNN/DNN implementation on EVEs and C66x DSPs in AM57x SoCs. Refer to the TIDL API User's Guide for details: http://software-dl.ti.com/mctools/esd/docs/tidl-api/index.html

Debian Build
---------------------------

```bash
sudo apt update
sudo apt install ti-opencl libboost-dev libopencv-core-dev libopencv-imgproc-dev libopencv-highgui-dev libjson-c-dev

git clone https://github.com/rcn-ee/tidl-api
cd tidl-api/
git checkout origin/v01.02.02-bb.org -b v01.02.02-bb.org
make -j2 build-api

sudo mkdir -p /usr/share/ti/tidl
sudo chown -R 1000:1000 /usr/share/ti/tidl/

make -j2 build-examples
```

Debian Porting...
---------------------------

```bash

sed -i -e 's:<CL/cl.h>:<CL/TI/cl.h>:g' tidl_api/src/ocl_device.h
sed -i -e 's:<CL/cl_ext.h>:<CL/TI/cl_ext.h>:g' tidl_api/src/ocl_device.h

sed -i -e 's:-lOpenCL:-lTIOpenCL:g' tidl_api/make.inc
sed -i -e 's:-lOpenCL:-lTIOpenCL:g' tidl_api/Makefile
sed -i -e 's:-lOpenCL:-lTIOpenCL:g' examples/make.common

```
