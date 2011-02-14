ASLIB     = libautoserial.a

CXXFLAGS += -Wall -O3 -I./include

SRCS     = as_buffer.cpp      \
	   as_file.cpp        \
	   as_text.cpp        \
	   binaryhandlers.cpp \
           classfactory.cpp   \
	   comparator.cpp     \
	   opaque.cpp

OBJ      = $(SRCS:.cpp=.o)

$(ASLIB): $(OBJ)
	ar r $@ $(OBJ)
	ranlib $@

%o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@

clean:
	rm -rf $(OBJ) $(ASLIB)
