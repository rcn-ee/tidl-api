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

MAJOR_VER=0
MINOR_VER=1
PATCH_VER=0

ifeq ($(shell git rev-parse --short HEAD 2>&1 1>/dev/null; echo $$?),0)
BUILD_SHA?=$(shell git rev-parse --short HEAD)
endif

LIB_NAME = tinn_api.a
DSP_SRCFILE = dsp/ocl_wrapper.cl
DSP_OUTFILE = dsp/ocl_wrapper.dsp_h
LIB_IMGUTIL_NAME = tinn_imgutil.a

all: $(LIB_NAME) $(LIB_IMGUTIL_NAME)

include make.inc
RM = rm
AR = ar


SRCS = ocl_device.cpp configuration_parser.cpp configuration.cpp\
	   executor.cpp execution_object.cpp trace.cpp util.cpp
SRCS_IMGUTIL = imgutil.cpp

OBJS = $(SRCS:.cpp=.o)
OBJS_IMGUTIL = $(SRCS_IMGUTIL:.cpp=.o)

HOST_SRC_FILES = $(addprefix src/,$(SRCS))
HOST_OBJ_FILES = $(addprefix obj/,$(OBJS))
HOST_SRC_IMGUTIL_FILES = $(addprefix src/,$(SRCS_IMGUTIL))
HOST_OBJ_IMGUTIL_FILES = $(addprefix obj/,$(OBJS_IMGUTIL))

HEADERS  = src/common_defines.h src/executor_impl.h src/ocl_device.h
HEADERS += src/parameters.h src/tidl_create_params.h src/trace.h src/util.h
HEADERS += inc/configuration.h inc/execution_object.h inc/executor.h
HEADERS += inc/imgutil.h

.PHONY: $(BUILD_ID)
BUILD_ID := -D_BUILD_VER=$(shell echo "" | \
                awk '{ printf ("%02d.%02d.%02d", $(MAJOR_VER), \
                $(MINOR_VER), $(PATCH_VER)); }') \
			-D_BUILD_SHA=$(BUILD_SHA)


ifeq ($(BUILD), debug)
	CXXFLAGS += -Og -g -ggdb
else
	CXXFLAGS += -O3
endif

CXXFLAGS += -std=c++11 -Wall -Werror -Wno-error=ignored-attributes
CXXFLAGS += -I$(TI_OCL_INSTALL)/usr/share/ti/opencl
CXXFLAGS += -I$(TARGET_ROOTDIR)/usr/share/ti/opencl
CXXFLAGS += -Isrc -Iinc
CXXFLAGS += $(BUILD_ID)

$(DSP_OUTFILE): $(DSP_SRCFILE)
	$(MAKE) -C dsp

src/ocl_device.cpp: $(DSP_OUTFILE)

obj/%.o: src/%.cpp $(HEADERS)
	@mkdir -p obj
	@echo Compiling $< ...
	$(CXX) -c $(CXXFLAGS) $< -o $@

$(LIB_NAME): $(HOST_OBJ_FILES)
	$(AR) cr $@ $(HOST_OBJ_FILES)


$(LIB_IMGUTIL_NAME): CXXFLAGS += $(shell pkg-config --cflags opencv)

$(LIB_IMGUTIL_NAME): $(HOST_OBJ_IMGUTIL_FILES)
	$(AR) cr $@ $(HOST_OBJ_IMGUTIL_FILES)

clean::
	$(MAKE) -C dsp clean
	$(RM) -f $(LIB_NAME) $(HOST_OBJ_FILES)
	$(RM) -f $(LIB_IMGUTIL_NAME) $(HOST_OBJ_IMGUTIL_FILES)
	$(RM) -rf obj

