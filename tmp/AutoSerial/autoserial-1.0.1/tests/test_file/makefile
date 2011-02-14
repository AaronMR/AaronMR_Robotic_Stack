LDFLAGS      = -L../../autoserial -lautoserial
CXXFLAGS     = -I../../include

default: test_file_binary

test_file_binary: test_file_binary.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

run: test_file_binary
	./$<

clean:
	rm -rf test_file_binary

