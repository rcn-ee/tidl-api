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

TARGET ?= arm

EXE = $(TARGET)/netviz

TINN_API_DIR=../tinn_api

include $(TINN_API_DIR)/make.inc

UNAME_M :=$(shell uname -m)

# -m32 required on x86 to ensure consistent layout of struct sTIDL_Network_t
ifeq ($(TARGET),x86)
	CXX = g++
	CXXFLAGS = -m32 -static
	LDFLAGS =
endif

CXXFLAGS += -o3
#CXXFLAGS += -g -ggdb
CXXFLAGS += --std=c++11 -Wall -Werror
CXXFLAGS += -Iinc -I$(TINN_API_DIR)/inc -I$(TINN_API_DIR)/src
SOURCES = main.cpp src/tinn_utils.cpp src/dot_graph.cpp $(TINN_API_DIR)/src/util.cpp

$(EXE): $(TINN_LIB) $(HEADERS) $(SOURCES)
	mkdir -p $(TARGET)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(TINN_LIB) $(LDFLAGS) -o $@

clean::
	$(RM) $(EXE) *.dot
