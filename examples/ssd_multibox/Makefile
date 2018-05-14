EXE = ssd_multibox

include ../make.common

CXXFLAGS += $(shell pkg-config --cflags opencv)
LIBS     += $(shell pkg-config --libs --static opencv)

SOURCES = main.cpp ../segmentation/object_classes.cpp

$(EXE): $(TIDL_API_LIB) $(HEADERS) $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(TIDL_API_LIB) \
	    $(LDFLAGS) $(LIBS) -o $@

clean::
	$(RM) -f frame_*.png multibox_*.png
