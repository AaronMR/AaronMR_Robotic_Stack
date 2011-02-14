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

/*!	\file opaque.h
	\brief Opaque serialized object class

	The OpaqueObject class can be used to wrap any object into an opaque form that
	can be passed around several processes that do not know any of the
	types used in the object before finally deserializing it again.
	This is useful for passing objects through service graphs.
*/

#ifndef INCLUDED_OPAQUE_H
#define INCLUDED_OPAQUE_H

#include "base.h"
#include "buffer.h"

namespace autoserial
{
	//! Opaque serialized representation of an object
	/*! This class can be used to wrap any object into an opaque form that
		can be passed around several processes that do not know any of the
		types used in the object before finally deserializing it again.
		This is useful for passing objects through service graphs.
	*/
	class OpaqueObject : public ISerializable
	{
		AS_CLASSDEF(OpaqueObject)
		AS_MEMBERS
			//! Serialized data stream
			AS_PRIVATEITEM(internal::Buffer<UInt8>,data)
			//! Endian key
			AS_PRIVATEITEM(UInt8,endianKey)
		AS_CLASSEND;

	private:
		//! Current offset in buffer
		Size bufferOff;

		//! Overlay target
		ISerializable *overlayTarget;

	public:
		//! Default constructor
		OpaqueObject();

		//! Copy constructor
		OpaqueObject(const OpaqueObject &other);

		//! Destructor
		virtual ~OpaqueObject() { }

		//! Retrieve object
		/*! \return Pointer to deserialized object.
		*/
		ISerializable *get();

		//! Set object
		/*! \param obj Object to store
		*/
		Result set(const ISerializable *obj);

		//! Return object size (for tracing purposes)
		/*! \return Size of serialized object
		*/
		Size getSize() { return data.size(); }

		//! Compare serialized objects
		/*! \return true if both serialized forms are equal
		*/
		bool equals(const OpaqueObject *other) const
		{
			if(data.size()!=other->data.size())
				return false;
			return (memcmp(data.get(),other->data.get(),data.size())==0);
		}

		//! Return pointer to data
		UInt8 *getPtr() { return data.get(); }
	};
}

#endif

