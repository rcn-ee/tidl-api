EXE = test_tinn

include ../make.common

$(EXE): SOURCES = main.cpp multiple_executors.cpp

$(EXE): $(TINN_LIB) $(HEADERS) $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(TINN_LIB) $(LDFLAGS) $(LIBS) -o $@

