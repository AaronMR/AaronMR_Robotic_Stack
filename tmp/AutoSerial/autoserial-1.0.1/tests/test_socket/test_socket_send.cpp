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

//#define AS_SYS_TYPES
#include <autoserial/autoserial_socket.h>

#include "tcpsender.h"
#include "../objects.h"

using namespace autoserial;


// Test flipping binary socket writer/reader
void runAS_Sock_Flip(const char*hostname="127.0.0.1", int port=8000) {

	int sock=(int)(new TCPSender(hostname,port))->connect();

	if (sock==SOCKET_ERROR)
		return;

	// Send magic number to determine flippiness
	// A sendMagicNumber() must correspond to a readMagicNumber()
	sendMagicNumber(sock);

	SimpleObject so;
	so.intItem=0x12345678;

	autoserial::BinarySocketWriter sw(sock);
	std::cout << "Send SimpleObject with BinarySocketWriter" << std::endl;
	std::cout << "     intItem 0x" << std::hex << so.intItem << std::dec << std::endl;
	sw.write(&so);
}

// Example socket writer. Send a SimpleObject resp. ListObject
// A listener socket in hostname at port 'port' must accept connection before
// running this method
void runAS_Socket(const char* hostname="127.0.0.1",int port=8000, bool print=true) {
	int sock=(int)(new TCPSender(hostname,port))->connect();

	if (sock==SOCKET_ERROR)
		return;

	BinarySocketWriter bsw(sock);

	SimpleObject so(10,20.0);
	ListObject lo;
	lo.init();

	std::cout << "Sending SimpleObject and ListObject!" << std::endl;
	if (print)
	{
		so.print();
		lo.print();
	}

	// Send SimpleObject
	if (AS_FAILED(bsw.write(&so)))
	{
		std::cout << "Could not send SimpleObject!" << std::endl;
		return;
	}

	// Send ListObject
	if (AS_FAILED(bsw.write(&lo)))
	{
		std::cout << "Could not send ListObject!" << std::endl;
		return;
	}
}


int main(int argc, char *argv[])
{
	std::cout << " TEST FLIPPING " << std::endl;
	runAS_Sock_Flip("127.0.0.1",8000);

	std::cout << " TEST BINARY SOCKET READER/WRITER " << std::endl;
	runAS_Socket("127.0.0.1",8001);

	return 0;
}
