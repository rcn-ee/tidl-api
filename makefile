
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


# makefile for TI internal use

ifneq (,$(findstring 86, $(shell uname -m)))
DEST_DIR ?= $(CURDIR)/install/am57
endif

INSTALL_DIR_API = $(DEST_DIR)/usr/share/ti/tidl
INSTALL_DIR_EXAMPLES = $(DEST_DIR)/usr/share/ti/examples/tidl

CP_ARGS=-Prf --preserve=mode,timestamps --no-preserve=ownership

build-api:
	$(MAKE) -C tidl_api

build-examples: install-api
	$(MAKE) -C examples

install-api: build-api
	mkdir -p $(INSTALL_DIR_API)
	cp $(CP_ARGS) tidl_api $(INSTALL_DIR_API)/

install-examples: build-examples
	mkdir -p $(INSTALL_DIR_EXAMPLES)
	cp $(CP_ARGS) examples/* $(INSTALL_DIR_EXAMPLES)/

clean:
	$(MAKE) -C tidl_api	clean
