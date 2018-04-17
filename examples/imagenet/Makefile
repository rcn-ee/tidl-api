EXE = imagenet

include ../make.common

CXXFLAGS += $(shell pkg-config --cflags opencv)
LIBS     += $(shell pkg-config --libs --static opencv)

SOURCES = main.cpp imagenet_classes.cpp

$(EXE): $(TINN_LIB) $(TINN_LIB_IMGUTIL) $(HEADERS) $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(TINN_LIB) $(TINN_LIB_IMGUTIL) \
	    $(LDFLAGS) $(LIBS) -o $@

