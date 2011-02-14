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

/*!	\file opaque.cpp
	\brief Opaque serialized object class

	The OpaqueObject class can be used to wrap any object into an opaque form that
	can be passed around several processes that do not know any of the
	types used in the object before finally deserializing it again.
	This is useful for passing objects through service graphs.
*/

#include "../include/autoserial/defs.h"
#include "../include/autoserial/opaque.h"
#include "../include/autoserial/as_buffer.h"

namespace autoserial
{
	OpaqueObject::OpaqueObject()
	{
		UInt32 k=0x12345678;
		endianKey=*(UInt8*)&k;
	}

	OpaqueObject::OpaqueObject(const OpaqueObject &other)
	{
		endianKey=other.endianKey;
		data=other.data;
	}


	ISerializable *OpaqueObject::get()
	{
		ISerializable *target=NULL;
		if(data.size()>0)
		{
			UInt32 k=0x12345678;
			if(endianKey==*(UInt8*)&k)
			{
				BinaryBufferReader r(data);
				r.read(&target);
			}
			else
			{
				FlippingBinaryBufferReader r(data);
				r.read(&target);
			}
		}
		return target;
	}

	Result OpaqueObject::set(const ISerializable *o)
	{
		Result r=AS_OK;
		if(o)
		{
			BinaryBufferWriter w(data);
			r=w.write(o);
		}
		else
			data.resize(0);
		return r;
	}

}

