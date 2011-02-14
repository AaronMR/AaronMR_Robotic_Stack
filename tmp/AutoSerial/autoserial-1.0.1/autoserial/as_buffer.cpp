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

/*!	\file as_buffer.cpp
	\brief Binary data reader/writer
*/

#include "../include/autoserial/as_buffer.h"

namespace autoserial
{
	Result BinaryBufferWriter::flush()
	{
		Size ts=0;
		for(Size i=0;i<bufferCount;i++)
			ts+=buffers[i].length;
		target.resize(ts);

		UInt32 off=0;
		for(Size i=0;i<bufferCount;i++)
		{
			memcpy(&target[off],buffers[i].data,buffers[i].length);
			off+=buffers[i].length;
		}
		return AS_OK;
	}

	BinaryBufferWriter::BinaryBufferWriter(internal::Buffer<UInt8>& trg) : target(trg)
	{
	}

	Result FlippingBinaryBufferWriter::flush()
	{
		Size ts=0;
		for(Size i=0;i<bufferCount;i++)
			ts+=buffers[i].length;
		target.resize(ts);

		UInt32 off=0;
		for(Size i=0;i<bufferCount;i++)
		{
			memcpy(&target[off],buffers[i].data,buffers[i].length);
			off+=buffers[i].length;
		}
		return AS_OK;
	}

	FlippingBinaryBufferWriter::FlippingBinaryBufferWriter(internal::Buffer<UInt8>& trg) : target(trg)
	{
	}

	Result BinaryBufferReader::readBytes(void *buf, Size len)
	{
		memcpy(buf,&source[offset],len);
		offset+=len;
		return AS_OK;
	}

	BinaryBufferReader::BinaryBufferReader(internal::Buffer<UInt8>& src) : source(src)
	{
		offset=0;
	}

	Result FlippingBinaryBufferReader::readBytes(void *buf, Size len)
	{
		memcpy(buf,&source[offset],len);
		offset+=len;
		return AS_OK;
	}

	FlippingBinaryBufferReader::FlippingBinaryBufferReader(internal::Buffer<UInt8>& src) : source(src)
	{
		offset=0;
	}
}

