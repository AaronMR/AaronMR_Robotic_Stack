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

/*!	\file as_mpi.h
	\brief Binary data reader/writer

	Provides serialization functions for binary targets using MPI communication.
*/


#ifndef INCLUDED_AS_MPI_H
#define INCLUDED_AS_MPI_H


#include "autoserial.h"
#include <mpi.h>

#define MPI_BUFFER_LEN 512
#define MPI_FAILED(v) ((v)!=MPI_SUCCESS)

namespace autoserial
{
	//! Specialized BinaryWriter for MPI communication
	class MPIBinaryWriter : public BasicBinaryWriter
	{
	private:
		//! Destination process
		int dest;

		//! Communicator for sending messages
		MPI_Comm comm;

		int tag;

	protected:
		//! Flush the buffered data to output
		Result flush() {
			for(Size i=0;i<bufferCount;i++)
			{
				if (MPI_FAILED(MPI_Send(const_cast<void*>(buffers[i].data), buffers[i].length, MPI_BYTE, dest, tag, comm)))
				{
					printf("Failed send of buffer %d, size %d\n",i,buffers[i].length);
					return AS_MPISEND_FAIL;
				}
			}
			return AS_OK;
		}

	public:
		//! Constructor
		MPIBinaryWriter(int dest, MPI_Comm comm,int tag=0) {
			this->dest=dest;
			this->comm=comm;
			this->tag=tag;
		}
	};

	//! Specialized BinaryReader for MPI communication
	class MPIBinaryReader : public BasicBinaryReader
	{
	private:
		//! Sending process of data to read
		int source;

		//! Message tag
		int tag;

		//! Status object
		MPI_Status *st;

		//! Local buffer for MPI messages
		char *mpiBuffer;

		//! Buffer size
		int mpiBufferSize;

		//! Index of unused data
		int mpiBufferIndex;

		//! Special value for empty buffers
		static const int EMPTY_INDEX = -1;

		//! Communicator for receiving messages
		MPI_Comm comm;

	private:
		//! Reset reader
		void reset() {
			mpiBufferIndex=EMPTY_INDEX; 
			source=MPI_ANY_SOURCE;
		}

	protected:
		//! Flush the buffered data to output
		Result readBytes(void *buf, Size len) {
			int remainingLength = (int)len;
			char *outputBuffer = (char*)buf;
			while (remainingLength > 0)
			{
				// Check if there is data left in mpiBuffer
				if (mpiBufferIndex != EMPTY_INDEX)
				{
					int dataLeft = mpiBufferSize - mpiBufferIndex;
					if (dataLeft > remainingLength) // There is more data than we need
					{
						memcpy(outputBuffer, &mpiBuffer[mpiBufferIndex], remainingLength);
						mpiBufferIndex += remainingLength;
						return AS_OK;
					}
					else
					{
						memcpy(outputBuffer, &mpiBuffer[mpiBufferIndex], dataLeft);
						mpiBufferIndex = EMPTY_INDEX;
						outputBuffer += remainingLength;
						remainingLength -= dataLeft;
						// Next iteration exits loop if dataLeft was equal to remainingLength
						// and MPI_Recv the next message otherwise
					}
				}
				else // mpiBuffer is empty
				{
					int mpiMsgSize;
					if (MPI_FAILED(MPI_Probe(source, tag, comm, st)))
						return AS_MPIRECV_FAIL;
					if (MPI_FAILED(MPI_Get_count(st, MPI_BYTE, &mpiMsgSize)))
						return AS_MPIRECV_FAIL;
					source=st->MPI_SOURCE;

					// Receive directly into argument buffer if possible
					if (remainingLength >= mpiMsgSize)
					{
						if(MPI_FAILED(MPI_Recv(outputBuffer, mpiMsgSize, MPI_BYTE, source, st->MPI_TAG, comm, st)))
						{
							printf("MPI Receive error!\n");
							return AS_MPIRECV_FAIL;
						}
						remainingLength-=mpiMsgSize;
						// Next iteration exits if remaniningLength was equal to mpiMsgSize,
						// and receives next message otherwise
					}
					else
					{
						assert(mpiBufferIndex == EMPTY_INDEX);
						// We must use a temporary buffer to store the message
						if (mpiBufferSize < mpiMsgSize) // Allocate/resize buffer as needed
						{
							if (mpiBuffer == NULL)
								mpiBuffer = (char*)malloc(mpiMsgSize);
							else
								mpiBuffer = (char*)realloc(mpiBuffer, mpiMsgSize);
							mpiBufferSize = mpiMsgSize;
						}
						// Offset index
						mpiBufferIndex = mpiBufferSize - mpiMsgSize;

						if(MPI_FAILED(MPI_Recv(mpiBuffer+mpiBufferIndex, mpiMsgSize, MPI_BYTE, source, st->MPI_TAG, comm, st)))
						{
							printf("MPI Receive error!\n");
							return AS_MPIRECV_FAIL;
						}
						// Next iteration reads into buffer
					}
				}
			}

			return AS_OK;
		}

	public:

		//! Constructor
		MPIBinaryReader(int src, MPI_Comm comm, int tag, MPI_Status *st) { 
			reset();
			this->source=src;
			this->comm=comm;
			this->mpiBufferSize=0;
			this->mpiBuffer=NULL;
			this->tag=tag;
			this->st=st;
		} 

		//! Destructor
		~MPIBinaryReader() {
			if(mpiBuffer!=NULL)
				free(mpiBuffer);
		}
	};

	
	//! Specialized MPI communicator for serialized objects
	class AutoserialCommunicator : public MPI::Comm 
	{
	public:
	
		//! Constructor
		AutoserialCommunicator(MPI_Comm comm) : MPI::Comm(comm) {
			asComm=comm;
		}

		//! Send data to process dest
		Result Send(ISerializable &data, int dest, int tag)
		{
			MPIBinaryWriter mbw(dest,asComm,tag);
			return mbw.write(&data);
		}

		//! Receive an object from process src
		ISerializable* Recv(int src, int tag)
		{
			ISerializable *data;
			MPI_Status st;
			MPIBinaryReader mbr(src,asComm,tag,&st);
			if (AS_FAILED(mbr.read(&data)))
				return NULL;
			return data;
		}
		//! Receive an object from process src, with status
		ISerializable* Recv(int src, int tag, MPI::Status& st)
		{
			ISerializable *data;
			MPI_Status c_st=(MPI_Status)st;
			MPIBinaryReader mbr(src,asComm,tag,&c_st);
			if (AS_FAILED(mbr.read(&data)))
				return NULL;
			// Copy status content back into parameter
			st=c_st;
			return data;
		}

		//! Send data to process dest
		template<typename Type> Result Send(Type& data, int dest, int tag)
		{
			return Send(*(ISerializable*)&data,dest,tag);
		}

		//! Send data to process dest. Tag is Type index
		template<typename Type> Result Send(Type& data, int dest)
		{
			return Send(*(ISerializable*)&data,dest,(int)Type::getTypeIndex());
		}

		//! Receive data from process src
		template<typename Type> Type *Recv(int src, int tag)
		{
			Type *data;
			MPI_Status st;
			MPIBinaryReader mbr(src,asComm,tag,&st);
			if (AS_FAILED(mbr.read((ISerializable**)&data)))
				return NULL;
			return data;
		}

		//! Receive data from process src
		template<typename Type> Type *Recv(int src, int tag, MPI::Status &st)
		{
			Type *data;
			MPI_Status c_st=(MPI_Status)st;
			MPIBinaryReader mbr(src,asComm,tag,&c_st);
			if (AS_FAILED(mbr.read((ISerializable**)&data)))
				return NULL;
			return data;
		}

		//! Receive data from process src. Tag is Type index.
		template<typename Type> Type *Recv(int src)
		{
			Type *data;
			MPIBinaryReader mbr(src,asComm,(int)Type::getTypeIndex(),MPI_STATUS_IGNORE);
			if (AS_FAILED(mbr.read((ISerializable**)&data)))
				return NULL;
			return data;
		}

		//! Receive data from process src. Tag is Type index.
		template<typename Type> Type *Recv(int src, MPI::Status &st)
		{
			Type *data;
			MPI_Status c_st=(MPI_Status)st;
			MPIBinaryReader mbr(src,asComm,(int)Type::getTypeIndex(),&c_st);
			if (AS_FAILED(mbr.read((ISerializable**)&data)))
				return NULL;
			return data;
		}

	private:

		//! MPI communicator 
		MPI_Comm asComm;

		//! Return a clone of this object. Definition of virtual method.
		AutoserialCommunicator &Clone(void) const
		{
			return *(new AutoserialCommunicator(asComm));
		}
	};
}

// C interface

//! Send data to process dest within the communicator comm
int AS_MPI_Send(autoserial::ISerializable &data, int dest, int tag, MPI_Comm comm) {
	autoserial::MPIBinaryWriter mbw(dest,comm,tag);
	return mbw.write(&data);
}

//! Receive data from process src within the communicator comm
int AS_MPI_Recv(autoserial::ISerializable *&data, int src, int tag, MPI_Comm comm, MPI_Status *st) {
	if(st!=MPI_STATUS_IGNORE)
	{
		autoserial::MPIBinaryReader mbr(src,comm,tag,st);
		return mbr.read(&data);
	}
	else
	{
		MPI_Status st;
		autoserial::MPIBinaryReader mbr(src,comm,tag,&st);
		return mbr.read(&data); // st is used within read()
	}
}

#endif
