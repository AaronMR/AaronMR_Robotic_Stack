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

/*!	\file reftypes.h
	\brief Various types for handling ISerializable *

	Pointer to single items, and vector of items
*/

#ifndef INCLUDED_REFTYPES_H
#define INCLUDED_REFTYPES_H

#include "base.h"

namespace autoserial
{

	//! Serializable vector class
	/*! This class can be used to serialize a vector of pointers to objects 
		derived  from ISerializable. The reference count of the objects is 
		<i>not</i> increased on assignment, but decreased on destruction 
		(i.e. VectorRef takes ownership of the assigned objects). If you 
		wish to keep the references as well, do not forget to addRef them.
		Objects that are removed from the vector also do <i>not</i> have
		their reference count updated. Therefore it is also the developers
		responsibility to be careful with these.
	*/
	template<typename RefType> class VectorRef : public std::vector<RefType*>
	{
	public:
		AS_TYPENAMETEMPLATE1(VectorRef,RefType);

		//! Destructor
		/*! The assigned objects are released on destruction.
		*/
		~VectorRef() 
		{ 
			for(typename std::vector<RefType*>::iterator it=this->begin();it!=this->end();++it)
			{
				if(*it!=NULL)
					(*it)->release();
			}
		}

		//! Write to storage
		/*! The class is not equivalent to std::vector when serializing, since
			the vector contains pointers, and pointers cannot normally be
			serialized. Therefore we have this special case here that handles
			pointers properly.
		*/
		Result write(IOutputStream *s) const 
		{ 
			ENTERITEM("size");
			XPSize sz=(XPSize)this->size();
			if(DPS_FAILED(typehandler::write(s,sz)))
				return DPS_FAIL;
			LEAVEITEM("size");
			for(typename std::vector<RefType*>::const_iterator i=this->begin();i!=this->end();i++)
			{
				ENTERITEM("element");
				// We write a reference to the pointed element
				if(DPS_FAILED(s->writeRef((RefType*)*i)))
					return DPS_FAIL;
				LEAVEITEM("element");
			}
			return DPS_OK;
		}

		//! Read from stream
		/*! The class is not equivalent to std::vector when deserializing, since
			the vector contains pointers, and pointers cannot normally be
			deserialized. Therefore we have this special case here that handles
			pointers properly.
		*/
		Result read(IInputStream *s) 
		{ 
			this->clear();
			ENTERITEM("size");
			XPSize sz;
			if(DPS_FAILED(typehandler::read(s,sz)))
				return DPS_FAIL;
			LEAVEITEM("size");
			for(Size i=0;i<sz;i++)
			{
				ENTERITEM("element");
				ISerializable *t;
				// We read a reference to the pointed element
				if(DPS_FAILED(s->readRef(&t)))
					return DPS_FAIL;
				this->push_back((RefType*)t);
				LEAVEITEM("element");
			}
			return DPS_OK;
		}
	};

	//! Serializable map class
	/*! This class can be used to serialize a map of pointers to objects 
		derived  from ISerializable. The reference count of the objects is 
		<i>not</i> increased on assignment, but decreased on destruction 
		(i.e. MapRef takes ownership of the assigned objects). If you 
		wish to keep the references as well, do not forget to addRef them.
	*/
	template<typename KeyType, typename RefType> class MapRef : public std::map<KeyType,RefType*>
	{
	public:
		AS_TYPENAMETEMPLATE2(MapRef,KeyType,RefType);

		//! Destructor
		/*! The assigned objects are released on destruction.
		*/
		~MapRef() 
		{ 
			for(typename std::map<KeyType,RefType*>::iterator it=this->begin();it!=this->end();++it)
			{
				if(it->second)
					it->second->release();
			}
		}

		//! Write to storage
		/*! The class is not equivalent to std::map when serializing, since
			the map contains pointers, and pointers cannot normally be
			serialized. Therefore we have this special case here that handles
			pointers properly.
		*/
		Result write(IOutputStream *s) const 
		{ 
			ENTERITEM("size");
			XPSize sz=(XPSize)this->size();
			if(DPS_FAILED(typehandler::write(s,sz)))
				return DPS_FAIL;
			LEAVEITEM("size");
			for(typename std::map<KeyType,RefType*>::const_iterator i=this->begin();i!=this->end();i++)
			{
				ENTERITEM("key");
				if(DPS_FAILED(typehandler::write(s,i->first)))
					return DPS_FAIL;
				LEAVEITEM("key");
				ENTERITEM("element");
				// We write a reference to the pointed element
				if(DPS_FAILED(s->writeRef(i->second)))
					return DPS_FAIL;
				LEAVEITEM("element");
			}
			return DPS_OK;
		}

		//! Read from stream
		/*! The class is not equivalent to std::map when deserializing, since
			the map contains pointers, and pointers cannot normally be
			deserialized. Therefore we have this special case here that handles
			pointers properly.
		*/
		Result read(IInputStream *s) 
		{ 
			this->clear();
			ENTERITEM("size");
			XPSize sz;
			if(DPS_FAILED(typehandler::read(s,sz)))
				return DPS_FAIL;
			LEAVEITEM("size");
			for(Size i=0;i<sz;i++)
			{
				KeyType k;
				ENTERITEM("key");
				if(DPS_FAILED(typehandler::read(s,k)))
					return DPS_FAIL;
				LEAVEITEM("key");
				ENTERITEM("element");
				ISerializable *t;
				// We read a reference to the pointed element
				if(DPS_FAILED(s->readRef(&t)))
					return DPS_FAIL;
				this->insert(std::pair<KeyType,RefType*>(k,(RefType*)t));
				LEAVEITEM("element");
			}
			return DPS_OK;
		}
	};

}

#endif

