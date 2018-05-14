EXE = segmentation

include ../make.common

CXXFLAGS += $(shell pkg-config --cflags opencv)
LIBS     += $(shell pkg-config --libs --static opencv)

SOURCES = main.cpp object_classes.cpp

$(EXE): $(TIDL_API_LIB) $(HEADERS) $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(TIDL_API_LIB) \
	    $(LDFLAGS) $(LIBS) -o $@

clean::
	$(RM) -f frame_*.png overlay_*.png
