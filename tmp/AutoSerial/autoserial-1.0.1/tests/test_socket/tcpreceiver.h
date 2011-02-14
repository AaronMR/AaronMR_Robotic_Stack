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


#ifndef _TCPRECEIVER_H_
#define _TCPRECEIVER_H_

#include <string.h>
#include <stdio.h>
#include <iostream>

#include <autoserial/autoserial_socket.h>

class TCPReceiver {

public:

	TCPReceiver(int port=8000) { this->port=port;};

	~TCPReceiver() {
		if (sock!=SOCKET_ERROR) closesocket(sock);
		if (server!=SOCKET_ERROR) closesocket(server);
	}

	SOCKET connect() {

		sockaddr_in sin;

#ifdef AS_PLATFORM_WIN32
		// Initialization
		WSADATA wsa;
		WSAStartup(MAKEWORD(2,0), &wsa);
#endif

		// Create socket
		if((server=socket(AF_INET,SOCK_STREAM,0))==INVALID_SOCKET)
		{
			std::cout << "Could not create socket!" << std::endl;
			return SOCKET_ERROR;
		}

		// Configure socket
		sin.sin_addr.s_addr = INADDR_ANY;
		sin.sin_family = AF_INET;
		sin.sin_port = htons(port);

		if (bind(server,(struct sockaddr*)&sin,sizeof(sin))==INVALID_SOCKET)
		{
			std::cout << "Could not bind port " << port << std::endl;
			return SOCKET_ERROR;
		}

		if (listen(server,5)==INVALID_SOCKET)
		{
			std::cout << "Could not listen!" << std::endl;
			return SOCKET_ERROR;
		}

		std::cout << "Wait for connection at port " << port << "...";
		sock=accept(server, NULL, NULL);
		if(sock==INVALID_SOCKET)
		{
			std::cout << "Connection error at sender!" << std::endl;
			return SOCKET_ERROR;
		}

		std::cout << "Successful socket creation!" << std::endl;

		return sock;
	}
	
private:
	SOCKET server;
	SOCKET sock;
	int port;
};
#endif
