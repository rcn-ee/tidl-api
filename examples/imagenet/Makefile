EXE = imagenet

include ../make.common

CXXFLAGS += $(shell pkg-config --cflags opencv)
LIBS     += $(shell pkg-config --libs opencv)

SOURCES = main.cpp imagenet_classes.cpp

$(EXE): $(TIDL_API_LIB) $(TIDL_API_LIB_IMGUTIL) $(HEADERS) $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(TIDL_API_LIB) $(TIDL_API_LIB_IMGUTIL) \
	    $(LDFLAGS) $(LIBS) -o $@

