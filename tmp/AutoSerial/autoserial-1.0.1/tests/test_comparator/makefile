LDFLAGS      = -L../../autoserial -lautoserial
CXXFLAGS     = -I../../include

default: test_comparator

test_comparator: test_comparator.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

run: test_comparator
	./$<

clean:
	rm -rf test_comparator

