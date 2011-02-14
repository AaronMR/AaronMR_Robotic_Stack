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

#ifndef _TCPSENDER_H_
#define _TCPSENDER_H_

#include <string.h>
#include <stdio.h>
#include <iostream>

#include <autoserial/autoserial_socket.h>

#define MAX_LENGTH 1048576

class TCPSender {

public:
	TCPSender(const char* hostname="127.0.0.1",int port=8000) {
		this->hostname=new char[strlen(hostname)+1];
		strcpy(this->hostname,hostname);
		this->port=port;
	};

	~TCPSender() {
		if (sock!=SOCKET_ERROR)
			closesocket(sock);
	}

	SOCKET connect() {
		sockaddr_in sockAddr;

#ifdef AS_PLATFORM_WIN32
		WSADATA wsaData;
		if(WSAStartup(MAKEWORD(2,0), &wsaData)!= 0)
		{
			std::cout << "Could not initialize Winsock API!" << std::endl;
			return SOCKET_ERROR;
		}
#endif

		if((sock=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP))==INVALID_SOCKET)
		{
			std::cout << "Could not create socket" << std::endl;
			return SOCKET_ERROR;
		}

		memset(&sockAddr,0x0,sizeof(sockAddr)); 

		sockAddr.sin_family = AF_INET;
		sockAddr.sin_port = htons(port);
		//sockAddr.sin_addr.S_un.S_addr = inet_addr(hostname);
		Int32 a,b,c,d;
		sscanf(hostname,"%d.%d.%d.%d",&a,&b,&c,&d);
		UInt32 address=htonl(d+(c>>8)+(b<<16)+(a<<24));
		sockAddr.sin_addr.s_addr = address;

		if(::connect(sock,(struct sockaddr *) &sockAddr, sizeof(sockAddr))!= 0) 
		{
			std::cout << "Could not connect socket at host " << hostname << " port " << port << std::endl;
			return SOCKET_ERROR;
		}
		return sock;
	}

	/*int send(const void *data, int len) {
		for(size_t cp=0;cp<len;cp+=MAX_LENGTH)
		{
			if(::send(sock,&((const char *)data)[cp],(int)min(len-cp,MAX_LENGTH),0)==SOCKET_ERROR)
				return -1;
		}
		return 0;
	}*/

private:
	SOCKET sock;
	int port;
	char* hostname;
};
#endif
