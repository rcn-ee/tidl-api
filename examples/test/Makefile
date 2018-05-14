EXE = test_tidl

include ../make.common

$(EXE): SOURCES = main.cpp multiple_executors.cpp

$(EXE): $(TIDL_API_LIB) $(HEADERS) $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(TIDL_API_LIB) $(LDFLAGS) $(LIBS) -o $@

