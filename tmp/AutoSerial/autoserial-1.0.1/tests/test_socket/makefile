ASDIR        = ../../autoserial
ASLIB        = -lautoserial

CXXFLAGS     = -Wall -I../../include

BINS         = test_socket_recv test_socket_send

all: $(BINS)

test_socket_recv: test_socket_recv.o
	$(CXX) -o $@ $< -L$(ASDIR) $(ASLIB)
	
test_socket_send: test_socket_send.o
	$(CXX) -o $@ $<  -L$(ASDIR) $(ASLIB)

%o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@

run: $(BINS)
	./test_socket_recv &
	sleep 1
	./test_socket_send
	wait
	
clean:
	rm -rf *.o $(BINS)

