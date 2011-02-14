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

/*!	\file as_socket.h
	\brief Binary data reader/writer

	Provides serialization functions for binary targets using TCP/IP sockets.
*/

#ifndef INCLUDED_AS_SOCKET_H
#define INCLUDED_AS_SOCKET_H

#include "binaryhandlers.h"

#ifdef AS_PLATFORM_WIN32

#pragma comment(lib,"ws2_32.lib")

#else

#include <sys/socket.h>
#include <netinet/in.h>

typedef int SOCKET;
#define SOCKET_ERROR -1
#define INVALID_SOCKET -1
#define closesocket(x) close(x)

#endif

namespace autoserial
{
	namespace socket_internal
	{
		const int SEGMENT_SIZE = 1048576;

		//! Send data to a socket
		/*! \param sock Socket on which to send
		\param data Pointer to send data
		\param len Length of data to send
		\return AS_OK if send was successful
		*/
		Result send(int sock, const void *data, Size len)
		{
			for(Size cp=0;cp<len;cp+=SEGMENT_SIZE)
			{
				if(::send(sock,&((const char *)data)[cp],(int)std::min(len-cp,(Size)SEGMENT_SIZE),0)==SOCKET_ERROR)
					return AS_SOCKET_ERROR;
			}
			return AS_OK;
		}

		//! Receive data from a socket
		/*! This blocking implementation waits until all the requested data has
		arrived.
		\param sock Socket on which to receive
		\param data Pointer to received data
		\param len Length of data to receive
		\return AS_OK if receive was successful
		*/
		Result recv(int sock, void *data, Size len)
		{
			Size tc=0;
			char *b=(char *)data;

			while(tc<len)
			{
				Int32 bc=::recv(sock,&b[tc],(int)std::min(len-tc,(Size)SEGMENT_SIZE),0);
				if(bc==0 || bc==SOCKET_ERROR)
					break;
				tc+=bc;
			}
			if(tc<len)
			{
				return AS_SOCKET_ERROR;
			}
			return AS_OK;
		}
	}

	
	//! Specialized BinaryWriter for TCP sockets
	class BinarySocketWriter : public BasicBinaryWriter
	{
	private:
		//! Socket to write to
		int sock;

	protected:
		//! Flush the buffered data to output
		virtual Result flush()
		{
#ifdef AS_PLATFORM_WIN32
			DWORD bs;
			WSASend(sock,(WSABUF*)(BasicBinaryWriter::DataBlock *)buffers,bufferCount,&bs,0,NULL,NULL);
#else
			Result r;
			for(Size i=0;i<bufferCount;i++)
			{
				if(AS_FAILED(r=(Result)socket_internal::send(sock,buffers[i].data,buffers[i].length)))
					return r;
			}
#endif
			return AS_OK;
		}

	public:
		//! Constructor
		BinarySocketWriter(int sock) { this->sock=sock; }
	};

	//! Specialized BinaryWriter for TCP sockets with endian flipping
	class FlippingBinarySocketWriter : public FlippingBasicBinaryWriter
	{
	private:
		//! Socket to write to
		int sock;

	protected:
		//! Flush the buffered data to output
		virtual Result flush()
		{
#ifdef AS_PLATFORM_WIN32
			DWORD bs;
			WSASend(sock,(WSABUF*)(FlippingBasicBinaryWriter::DataBlock *)buffers,bufferCount,&bs,0,NULL,NULL);
#else
			Result r;
			for(Size i=0;i<bufferCount;i++)
			{
				if(AS_FAILED(r=(Result)socket_internal::send(sock,buffers[i].data,buffers[i].length)))
					return r;
			}
#endif
			return AS_OK;
		}

	public:
		//! Constructor
		FlippingBinarySocketWriter(int sock) { this->sock=sock; }
	};


	//! Specialized BinaryReader TCP sockets
	class BinarySocketReader : public BasicBinaryReader
	{
	private:
		//! Socket to read from
		int sock;

		//! Local buffer for items below 4k
		UInt8 buffer[4096];

		//! Buffer index
		Size bufferIndex;

		//! Buffer size
		Size bufferSize;

		//! Buffer error state
		bool bufferValid;

	protected:
		//! Flush the buffered data to output
		virtual Result readBytes(void *buf, Size len)
		{
			if(bufferSize)
			{
				if(!bufferValid)
					return AS_SOCKET_ERROR;
				if((Size)(bufferSize-bufferIndex)>=len)
				{
					memcpy(buf,&buffer[bufferIndex],len);
					bufferIndex+=len;
					if(bufferIndex>=bufferSize)
						bufferIndex=bufferSize=0;
					return AS_OK;
				}
				else
				{
					Size rl=len-(bufferSize-bufferIndex);
					memcpy(buf,&buffer[bufferIndex],bufferSize-bufferIndex);
					bufferIndex=bufferSize=0;
					return socket_internal::recv(sock,((UInt8*)buf)+len-rl,rl);
				}
			}
			else
				return socket_internal::recv(sock,buf,len);
		}

		//! Use cache hint
		virtual void cacheHint(Size len)
		{
			bufferSize=len>4096?4096:len;
			if(AS_FAILED(socket_internal::recv(sock,buffer,bufferSize)))
			{
				bufferValid=false;
				return;
			}
			bufferIndex=0;
			bufferValid=true;
		}

	public:
		//! Constructor
		BinarySocketReader(int sock) { this->sock=sock; bufferIndex=0; bufferSize=0; bufferValid=true; }
	};

	//! Specialized FlippingBinaryReader for TCP sockets with endian flipping
	class FlippingBinarySocketReader : public FlippingBasicBinaryReader
	{
	private:
		//! Socket to read from
		int sock;

	protected:
		//! Flush the buffered data to output
		virtual Result readBytes(void *buf, Size len)
		{
			return socket_internal::recv(sock,buf,len);
		}

	public:
		//! Constructor
		FlippingBinarySocketReader(int sock) { this->sock=sock; }
	};

	
	//! Send a magic number to determine flippiness. 
	/* Receive is completed by the method readMagicNumber() which tell whether
	flipping is performed or not */
	Result sendMagicNumber(int sock)
	{
		UInt32 endianKey=0x12345678;
		return socket_internal::send(sock,&endianKey,4);
	}

	enum FlipResult { FLIP, NOFLIP, UNKNOWNFLIP };

	//! Tell whether magic number was flipped or not
	FlipResult readMagicNumber(int sock)
	{
		FlipResult fr=UNKNOWNFLIP;
		UInt32 endianKey;
		if (AS_FAILED(socket_internal::recv(sock,&endianKey,4)))
			return fr;
		if(endianKey==0x12345678)
			fr=NOFLIP;
		else if(endianKey==0x78563412)
			fr=FLIP;
		return fr;
	}
}

#endif
