LDFLAGS      = -L../../autoserial -lautoserial
CXXFLAGS     = -I../../include

default: test_MPI

test_MPI: test_MPI.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

run: test_MPI # Runs with 3 processes
	$(MPIEXEC) 3 ./$<

clean:
	rm -rf test_MPI

