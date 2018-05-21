EXE = ssd_multibox

include ../make.common

LIBS     += -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio\
			-lopencv_imgproc -lopencv_core

SOURCES = main.cpp ../segmentation/object_classes.cpp

$(EXE): $(TIDL_API_LIB) $(HEADERS) $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(TIDL_API_LIB) $(LDFLAGS) $(LIBS) -o $@

clean::
	$(RM) -f frame_*.png multibox_*.png
