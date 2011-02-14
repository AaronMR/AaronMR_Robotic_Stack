/*
	autoserial
	A cross-platform serialization and reflexion library for C++ objects
	Copyright (C) 2000-2008:
		
		Sebastian Gerlach
		Kenzan Technologies
			(http://www.kenzantech.com)

		Basile Schaeli
			(basile schaeli at a3 epfl ch)

		Mamy Fetiarison

		Peripheral Systems Laboratory
		Ecole Polytechnique Fédérale de Lausanne (EPFL)
			(http://lsp.epfl.ch)
	
	All rights reserved.
	
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
		* Redistributions of source code must retain the above copyright
		  notice, this list of conditions and the following disclaimer.
		* Redistributions in binary form must reproduce the above copyright
		  notice, this list of conditions and the following disclaimer in the
		  documentation and/or other materials provided with the distribution.
		* Neither the names of "Kenzan Technologies", "Peripheral Systems Laboratory", 
		  "Ecole Polytechnique Fédérale de Lausanne (EPFL)", nor the names of
		  the contributors may be used to endorse or promote products derived
		  from this software without specific prior written permission.
	
	THIS SOFTWARE IS PROVIDED BY COPYRIGHT HOLDERS ``AS IS'' AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/



#include <autoserial/autoserial_socket.h>
#include "../objects.h"

#include "tcpreceiver.h"
#include <stdio.h>

using namespace autoserial; 

// Test of flipping writer/reader with sockets
// Another process must send magic number with sendMagicNumber
// this process 
void runAS_Sock_Flip(int port=8000) {
	SimpleObject *so;
	BasicBinaryReader *bbr;

	int sock=(int)(new TCPReceiver(port))->connect();

	if (sock==SOCKET_ERROR) 
		return;

	// Read magic number to determine flippiness
	switch (readMagicNumber(sock))
	{
	case FLIP:
		std::cout << "Flipping performed!" << std::endl;
		bbr=new FlippingBinarySocketReader(sock);
		break;
	case NOFLIP:
		std::cout << "No flipping!" << std::endl;
		bbr=new BinarySocketReader(sock);
		break;
	case UNKNOWNFLIP:
		std::cout << "Dont know flippiness!" << std::endl;
		return;
	}

	if (AS_FAILED(bbr->read((ISerializable**)&so)))
	{
		std::cout << "Could not read SimpleObject from socket!" << std::endl;
			return;
	}
	std::cout << "Received SimpleObject with (no flipping) BinarySocketReader " << std::endl;
	std::cout << "         intItem 0x" << std::hex << so->intItem << std::dec << std::endl ;

	delete bbr;
}


// Example socket reader
// Wait for receiving SimpleObject and ListObject
void runAS_Socket(int port=8000, bool print=true) {
	int sock=(int)(new TCPReceiver(port))->connect();

	if (sock==SOCKET_ERROR) 
		return;

	BinarySocketReader bsr(sock);

	// Receive SimpleObject
	SimpleObject *so;
	if(AS_FAILED(bsr.read((ISerializable**)&so)))
		std::cout << "Error at receiving SimpleObject" << std::endl;

	// Receive List
	ListObject *lo;
	if(AS_FAILED(bsr.read((ISerializable**)&lo)))
		std::cout << "Error at receiving ListObject" << std::endl;

	std::cout << "Sucessfull receive of SimpleObject and ListObject" << std::endl;
	if (print) {
		so->print();
		lo->print();
	}
}

int main(int argc, int *argv[])
{
	std::cout << " TEST FLIPPING " << std::endl;
	runAS_Sock_Flip(8000);

	std::cout << " TEST BINARY SOCKET READER/WRITER " << std::endl;
	runAS_Socket(8001);
	return 0;
}

