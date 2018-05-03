EXE = ssd_multibox

include ../make.common

CXXFLAGS += $(shell pkg-config --cflags opencv)
LIBS     += $(shell pkg-config --libs --static opencv)

SOURCES = main.cpp ../segmentation/object_classes.cpp

$(EXE): $(TINN_LIB) $(HEADERS) $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(TINN_LIB) \
	    $(LDFLAGS) $(LIBS) -o $@

clean::
	$(RM) -f frame_*.png multibox_*.png
