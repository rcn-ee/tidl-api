EXE = test_tinn

RM = rm
AR = ar
CP = cp

TINN_DIR = ../../tinn_api
TINN_LIB_NAME = tinn_api.a
TINN_LIB = $(TINN_DIR)/$(TINN_LIB_NAME)

DSP_OUTFILE = $(TINN_DIR)/dsp/ocl_wrapper.out

include $(TINN_DIR)/make.inc

ifeq ($(BUILD), debug)
	CXXFLAGS += -Og -g -ggdb
else
	CXXFLAGS += -O3
endif

CXXFLAGS += -I. -I$(TINN_DIR)/inc -std=c++11

HEADERS = 
LIBS    = -lOpenCL -locl_util -lpthread

$(EXE): SOURCES = main.cpp multiple_executors.cpp

$(EXE): $(TINN_LIB) $(HEADERS) $(DSP_OUTFILE) $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(TINN_LIB) $(LDFLAGS) $(LIBS) -o $@

.PHONY: $(DSP_OUTFILE)
$(DSP_OUTFILE):
	$(MAKE) -C $(TINN_DIR)/dsp
	$(CP) $(DSP_OUTFILE) .

.PHONY: $(TINN_LIB)
$(TINN_LIB):
	$(MAKE) -C $(TINN_DIR) $(TINN_LIB_NAME)

clean::
	$(RM) -f $(EXE) stats_tool_out.* *.out
	$(MAKE) -C $(TINN_DIR) clean
	$(MAKE) -C $(TINN_DIR)/dsp clean
