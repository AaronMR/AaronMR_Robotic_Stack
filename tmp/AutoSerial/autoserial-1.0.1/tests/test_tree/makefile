LDFLAGS      = -L../../autoserial -lautoserial 
CXXFLAGS     = -Wall -I../../include -g

default: test_tree

test_tree: test_tree.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

run: test_tree
	./$<

clean:
	rm -rf test_tree

