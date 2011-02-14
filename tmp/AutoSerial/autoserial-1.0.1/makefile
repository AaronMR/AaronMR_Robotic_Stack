
# C++ compiler
export CXX=g++ 

# MPI specific variables
# MPI C++ compiler wrapper
MAKEARGSMPI="CXX=mpicxx"
RUNMPI="MPIEXEC=mpirun -n"

# Build libautoserial.a
aslib:
	cd autoserial; ${MAKE}


# Build and run test programs
test: aslib
	cd tests; ${MAKE} $(MAKEARGS)
	@echo "Test programs compiled successfully"
	cd tests; ${MAKE} run


# Build and run the mpi test
testMPI: aslib
	cd tests/test_MPI; ${MAKE} $(MAKEARGSMPI)
	@echo "Test MPI program compiled successfully"
	cd tests/test_MPI; ${MAKE} $(RUNMPI) run
	


# Compile everything and run all tests
all: aslib test testMPI


clean:
	cd autoserial; ${MAKE} clean
	cd tests; ${MAKE} clean
