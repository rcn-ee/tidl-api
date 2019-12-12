# Copyright (c) 2018 Texas Instruments Incorporated - http://www.ti.com/
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of Texas Instruments Incorporated nor the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
PYTHON_INCLUDE_DIR ?= $(wildcard $(TARGET_ROOTDIR)/usr/include/python3*)

LIB_NAME = tidl_api.a
SHARED_LIB_NAME = libtidl_api.so
LIB_IMGUTIL_NAME = tidl_imgutil.a
SHARED_LIB_IMGUTIL_NAME = libtidl_imgutil.so
PY_LIB_NAME = tidl.so

all: $(LIB_NAME) $(LIB_IMGUTIL_NAME) $(PY_LIB_NAME) \
     $(SHARED_LIB_NAME) $(SHARED_LIB_IMGUTIL_NAME)

include make.inc
include make.buildid

SO_VER=$(MAJOR_VER).$(MINOR_VER).$(PATCH_VER)

RM = rm
AR = ar


SRCS = ocl_device.cpp configuration_parser.cpp configuration.cpp\
	   executor.cpp execution_object.cpp trace.cpp util.cpp \
       execution_object_pipeline.cpp \
       subgraph_runtime.cpp subgraph_data_conv.cpp
SRCS_IMGUTIL = imgutil.cpp
SRCS_PYBIND  = pybind_eo.cpp pybind_eop.cpp pybind_executor.cpp \
			   pybind_configuration.cpp pybind_helpers.cpp

OBJS = $(SRCS:.cpp=.o)
OBJS_IMGUTIL = $(SRCS_IMGUTIL:.cpp=.o)
OBJS_PYBIND  = $(SRCS_PYBIND:.cpp=.o)

HOST_OBJ_FILES         = $(addprefix obj/,$(OBJS))
HOST_OBJ_IMGUTIL_FILES = $(addprefix obj/,$(OBJS_IMGUTIL))
HOST_OBJ_PYBIND_FILES  = $(addprefix obj/,$(OBJS_PYBIND))

HEADERS  = src/common_defines.h src/executor_impl.h src/ocl_device.h
HEADERS += src/parameters.h src/tidl_create_params.h src/trace.h src/util.h
HEADERS += inc/configuration.h inc/execution_object.h inc/executor.h
HEADERS += inc/imgutil.h src/device_arginfo.h inc/execution_object_pipeline.h

ifeq ($(BUILD), debug)
	CXXFLAGS += -Og -g -ggdb
else
	CXXFLAGS += -O3
endif

CXXFLAGS += -std=c++11 -Wall -Werror -Wno-error=ignored-attributes
CXXFLAGS += -fPIC
CXXFLAGS += -I$(TI_OCL_INSTALL)/usr/share/ti/opencl
CXXFLAGS += -I$(TARGET_ROOTDIR)/usr/share/ti/opencl
CXXFLAGS += -Isrc -Iinc
CXXFLAGS += $(BUILD_ID)
PY_INCLUDE = -I$(PYTHON_INCLUDE_DIR) -I$(PYBIND11_INC_DIR)

# pybind11 recommends setting visibility to hidden to reduce code size and
# prevent name clashed when multiple shared libraries use pybind11
$(HOST_OBJ_PYBIND_FILES): CXXFLAGS += -fvisibility=hidden

$(HOST_OBJ_PYBIND_FILES): obj/%.o: src/%.cpp $(HEADERS) src/pybind_common.h
	@mkdir -p obj
	@echo Compiling pybind $< ...
	$(CXX) -c $(PY_INCLUDE) $(CXXFLAGS) $< -o $@


obj/%.o: src/%.cpp $(HEADERS)
	@mkdir -p obj
	@echo Compiling $< ...
	$(CXX) -c $(CXXFLAGS) $< -o $@

$(LIB_NAME): $(HOST_OBJ_FILES)
	$(AR) cr $@ $(HOST_OBJ_FILES)

$(SHARED_LIB_NAME): $(HOST_OBJ_FILES)
	$(CXX) $(CXXFLAGS) -Wl,-Bsymbolic -shared -Wl,-soname,$@.$(MAJOR_VER) \
		-lOpenCL $(HOST_OBJ_FILES) -o $@.$(SO_VER)
	ln -sf $@.$(SO_VER) $@.$(MAJOR_VER)
	ln -sf $@.$(MAJOR_VER) $@

$(PY_LIB_NAME): $(HOST_OBJ_PYBIND_FILES) $(LIB_NAME)
	$(CXX) $(CXXFLAGS) -Wl,-Bsymbolic -shared -lOpenCL -locl_util $^ -o $@

$(LIB_IMGUTIL_NAME): $(HOST_OBJ_IMGUTIL_FILES)
	$(AR) cr $@ $(HOST_OBJ_IMGUTIL_FILES)

$(SHARED_LIB_IMGUTIL_NAME): $(HOST_OBJ_IMGUTIL_FILES)
	$(CXX) $(CXXFLAGS) -Wl,-Bsymbolic -shared -Wl,-soname,$@.$(MAJOR_VER) \
		$(HOST_OBJ_IMGUTIL_FILES) -o $@.$(SO_VER)
	ln -sf $@.$(SO_VER) $@.$(MAJOR_VER)
	ln -sf $@.$(MAJOR_VER) $@

clean::
	$(RM) -f $(LIB_NAME) $(PY_LIB_NAME)
	$(RM) -f $(LIB_IMGUTIL_NAME)
	$(RM) -f $(SHARED_LIB_NAME)* $(SHARED_LIB_IMGUTIL_NAME)*
	$(RM) -rf obj

