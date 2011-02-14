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

/*!	\file as_buffer.h
	\brief Binary data reader/writer

	Provides serialization functions for memory blocks.
*/

#ifndef INCLUDED_AS_BUFFER_H
#define INCLUDED_AS_BUFFER_H

#include "binaryhandlers.h"
#include "buffer.h"

namespace autoserial
{
	//! Specialized BinaryWriter for memory blocks
	class BinaryBufferWriter : public BasicBinaryWriter
	{
	private:
		//! Buffer to write to
		internal::Buffer<UInt8>& target;

	protected:
		//! Flush the buffered data to output
		virtual Result flush();

	public:
		//! Constructor
		BinaryBufferWriter(internal::Buffer<UInt8>& target);
	};

	//! Specialized BinaryWriter for memory blocks with endian flipping
	class FlippingBinaryBufferWriter : public FlippingBasicBinaryWriter
	{
	private:
		//! Buffer to write to
		internal::Buffer<UInt8>& target;

	protected:
		//! Flush the buffered data to output
		virtual Result flush();

	public:
		//! Constructor
		FlippingBinaryBufferWriter(internal::Buffer<UInt8>& target);
	};

	//! Specialized BinaryReader for memory blocks
	class BinaryBufferReader : public BasicBinaryReader
	{
	private:
		//! Buffer to read from
		internal::Buffer<UInt8>& source;

		//! Current offset
		Size offset;

	protected:
		//! Flush the buffered data to output
		virtual Result readBytes(void *buf, Size len);

	public:
		//! Constructor
		BinaryBufferReader(internal::Buffer<UInt8>& target);
	};

	//! Specialized BinaryReader for memory blocks with endian flipping
	class FlippingBinaryBufferReader : public FlippingBasicBinaryReader
	{
	private:
		//! Buffer to read from
		internal::Buffer<UInt8>& source;

		//! Current offset
		Size offset;

	protected:
		//! Flush the buffered data to output
		virtual Result readBytes(void *buf, Size len);

	public:
		//! Constructor
		FlippingBinaryBufferReader(internal::Buffer<UInt8>& target);
	};
}

#endif

