TI Deep Learning (TIDL) API
---------------------------

TIDL API brings Deep Learning to the edge and enables Linux applications to leverage TI’s proprietary CNN/DNN implementation on EVEs and C66x DSPs in AM57x SoCs.  It requires OpenCL v1.1.15.1 or newer. Refer the User's Guide for details: http://software-dl.ti.com/mctools/esd/docs/tidl-api/index.html


Debian Build
---------------------------

```bash
sudo apt update
sudo apt install ti-opencl libboost-dev libopencv-core-dev libopencv-imgproc-dev libopencv-highgui-dev

git clone https://github.com/rcn-ee/tidl-api
cd tidl-api/
git checkout origin/v01.00.00.03-bb.org -b v01.00.00.03-bb.org
make -j2 build-api

sudo mkdir -p /usr/share/ti/tidl
sudo chown -R 1000:1000 /usr/share/ti/tidl/

make -j2 build-examples
```

