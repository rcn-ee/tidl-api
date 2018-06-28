
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


# makefile for building from the tidl-api git repo
# Cross-compilation requires TARGET_ROOTDIR to be set.
# E.g.
# PSDK_LINUX=<path to Processor Linux SDK install>
# TARGET_ROOTDIR=$PSDK_LINUX/linux-devkit/sysroots/armv7ahf-neon-linux-gnueabi

ifneq (,$(findstring 86, $(shell uname -m)))
DEST_DIR ?= $(CURDIR)/install/am57
VIEWER_TARGET=x86
else
VIEWER_TARGET=arm
endif

INSTALL_DIR_API = $(DEST_DIR)/usr/share/ti/tidl
INSTALL_DIR_EXAMPLES = $(DEST_DIR)/usr/share/ti/examples/tidl

CP_ARGS = -Prf
ifneq (,$(findstring 86, $(shell uname -m)))
CP_ARGS += --preserve=mode,timestamps --no-preserve=ownership
endif

build-api:
	$(MAKE) -C tidl_api

build-examples: install-api
	$(MAKE) -C examples

# Build HTML from Sphinx RST, requires Sphinx to be installed
build-docs:
	$(MAKE) -C docs

install-api: build-api
	mkdir -p $(INSTALL_DIR_API)
	cp $(CP_ARGS) tidl_api $(INSTALL_DIR_API)/

install-examples: build-examples
	mkdir -p $(INSTALL_DIR_EXAMPLES)
	cp $(CP_ARGS) examples/* $(INSTALL_DIR_EXAMPLES)/

build-viewer:
	$(MAKE) TARGET=$(VIEWER_TARGET) -C viewer

clean:
	$(MAKE) -C tidl_api	clean
	$(MAKE) -C examples	clean
