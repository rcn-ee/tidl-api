EXE = imagenet

include ../make.common

LIBS     += -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio\
			-lopencv_imgproc -lopencv_core

SOURCES = main.cpp imagenet_classes.cpp

$(EXE): $(TIDL_API_LIB) $(TIDL_API_LIB_IMGUTIL) $(HEADERS) $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(TIDL_API_LIB) $(TIDL_API_LIB_IMGUTIL) \
	    $(LDFLAGS) $(LIBS) -o $@

