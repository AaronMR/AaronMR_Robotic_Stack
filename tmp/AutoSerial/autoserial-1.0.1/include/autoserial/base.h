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

/*!	\file base.h
	\brief Base declarations for autoserial

	Basic declarations for types, interfaces and error handling.
*/

#ifndef INCLUDED_BASE_H
#define INCLUDED_BASE_H

#include "defs.h"
#include "classregistry.h"


namespace autoserial
{
	//! Return value for functions in interfaces
	enum Result {
				AS_OK,
				AS_FAIL,
				AS_SOCKET_ERROR,
				AS_FILE_ERROR,
				AS_MPI_FAIL,
				AS_MPISEND_FAIL,
				AS_MPIRECV_FAIL
			};

	# define AS_FAILED(v) ((v)!=autoserial::AS_OK)

	class ISerializable;

	//! Base class for output streams
	/*! This class serves as a base class for all output streams used by the 
		serializer. It must handle all simple C++ types.
	*/
	class IOutputStream
	{
	public:
		//! Destructor
		virtual ~IOutputStream() { }

		//! Write signed 8-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeInt8(const Int8& item) = 0;
		
		//! Write unsigned 8-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeUInt8(const UInt8& item) = 0;

		//! Write signed 16-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeInt16(const Int16& item) = 0;

		//! Write unsigned 16-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeUInt16(const UInt16& item) = 0;

		//! Write signed 32-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeInt32(const Int32& item) = 0;

		//! Write unsigned 32-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeUInt32(const UInt32& item) = 0;

		//! Write signed 64-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeInt64(const Int64& item) = 0;

		//! Write unsigned 64-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeUInt64(const UInt64& item) = 0;

		//! Write 32-bit floating point value
		/*! \param item Value to write
		*/
		virtual Result writeFloat(const Float& item) = 0;

		//! Write 64-bit floating point value
		/*! \param item Value to write
		*/
		virtual Result writeDouble(const Double& item) = 0;

		//! Write boolean value
		/*! \param item Value to write
		*/
		virtual Result writeBool(const Bool& item) = 0;

		//! Write STL string value
		/*! \param item Value to write
		*/
		virtual Result writeString(const std::string& item) = 0;

		//! Write an array of simple types
		/*! \param data Address of memory block containing values to write
			\param count Number of elements
			\param size Size of individual elements in bytes
		*/
		virtual Result writeArray(const void *data, const Size count, const Size size) = 0;

		//! Enter named item
		/*! This is used when AS_NAMED_SERIALIZATION is defined to enable
			rich output (for example XML).
			\param name Name of item to enter
		*/
		virtual Result enter(const char *name) = 0;

		//! Leave named item
		/*! This is used when AS_NAMED_SERIALIZATION is defined to enable
			rich output (for example XML).
			\param name Name of item to leave
		*/
		virtual Result leave(const char *name) = 0;

		//! Write a reference to an object
		/*! Stores an object in the output stream by reference. If the same instance of an object
			is serialized multiple times to the same stream, it is physically stored only once.
			\param item Object to write
		*/
		virtual Result writeRef(const ISerializable* item) = 0;

		//! Write a complete object to stream
		/*! This is the main entry point for serialization. Calling this function
			will write the object and all its dependencies to the output stream.
			\param item Object to write
		*/
		virtual Result write(const ISerializable* item) = 0;

		//! Write down object type to stream
		virtual Result type (const char *name) = 0;
	};

	//! Base class for input streams
	/*! This class serves as a base class for all output streams used by the 
		serializer. It must handle all simple C++ types.
	*/
	class IInputStream
	{
	public:
		//! Destructor
		virtual ~IInputStream() { }
 
		//! Read signed 8-bit integer
		/*! \param item Value to read
		*/
		virtual Result readInt8(Int8& item) = 0;

		//! Read unsigned 8-bit integer
		/*! \param item Value to read
		*/
		virtual Result readUInt8(UInt8& item) = 0;

		//! Read signed 16-bit integer
		/*! \param item Value to read
		*/
		virtual Result readInt16(Int16& item) = 0;

		//! Read unsigned 16-bit integer
		/*! \param item Value to read
		*/
		virtual Result readUInt16(UInt16& item) = 0;

		//! Read signed 32-bit integer
		/*! \param item Value to read
		*/
		virtual Result readInt32(Int32& item) = 0;

		//! Read unsigned 32-bit integer
		/*! \param item Value to read
		*/
		virtual Result readUInt32(UInt32& item) = 0;

		//! Read signed 64-bit integer
		/*! \param item Value to read
		*/
		virtual Result readInt64(Int64& item) = 0;

		//! Read signed 64-bit integer
		/*! \param item Value to read
		*/
		virtual Result readUInt64(UInt64& item) = 0;

		//! Read 32-bit floating point value
		/*! \param item Value to read
		*/
		virtual Result readFloat(Float& item) = 0;

		//! Read 64-bit floating point value
		/*! \param item Value to read
		*/
		virtual Result readDouble(Double& item) = 0;

		//! Read boolean value
		/*! \param item Value to read
		*/
		virtual Result readBool(Bool& item) = 0;

		//! Read STL string value
		/*! \param item Value to read
		*/
		virtual Result readString(std::string& item) = 0;

		//! Read an array of simple types
		/*! \param data Address of memory block containing values to read
			\param count Number of elements
			\param size Size of individual elements in bytes
		*/
		virtual Result readArray(void *data, const Size count, const Size size) = 0;

		//! Enter named item
		/*! This is used when AS_NAMED_SERIALIZATION is defined to enable
			rich output (for example XML).
			\param name Name of item to enter
		*/
		virtual Result enter(const char *name) = 0;
		
		//! Leave named item
		/*! This is used when AS_NAMED_SERIALIZATION is defined to enable
			rich output (for example XML).
			\param name Name of item to leave
		*/
		virtual Result leave(const char *name) = 0;

		//! Read a reference to an object
		/*! \param ref Pointer to a buffer that is to contain the reference to the read object
			\return AS_OK if successful, an error otherwise
		*/
		virtual Result readRef(ISerializable **ref) = 0;

		//! Read a complete object from stream
		/*! This is the main entry point for deserialization. Calling this function
			will read an object and all its dependencies from the input stream.
			\param obj Read object
		*/
		virtual Result read(ISerializable **obj) = 0;
	};

	class Comparator;

	/*! \brief Interface for handling object serialization/deserialization and
		reference counting.
	*/
	class ISerializable
	{
		AS_TYPENAMESTD(ISerializable);

	public:
		//! Destructor
		virtual ~ISerializable() { }

		//! Get the type of this object
		/*!	Returns a unique constant string identifying this object.
			\return Constant string pointer
		*/
		virtual const char *getTypeNameV() const = 0;

		//! Get the type index of this object
		/*!	Returns a unique constant integer identifying this object.
			\return Constant integer pointer
		*/
		virtual Size getTypeIndexV() const = 0;

		//! Get the class factory type index of this object
		/*!	Returns a unique constant integer identifying this object.
			\return Constant integer pointer
		*/
		virtual Size getFactoryTypeIndexV() const = 0;

		//! Write this object to a stream
		/*! \param stream Stream to which to write
			\return AS_OK if the object was written successfully
		*/
		virtual Result writeV(IOutputStream *stream) const = 0; 

		//! Read this object from a stream
		/*! \param stream Stream from which to read
			\return AS_OK if the object was read successfully
		*/
		virtual Result readV(IInputStream *stream) = 0; 

		//! Type comparison
		/*! \return true if the current type is the template argument,
			false otherwise.
		*/
		template<typename Ref> bool is() const { return Ref::getTypeIndex()==getTypeIndexV(); }
		//template<typename Ref> bool isSubclassOf() const { return Ref::baseClassAccessPoint::contains<Ref>(); }

		//! Object content comparison
		/*! \param Object to compare with
			\return true if parameter is equal to object
		*/
		virtual void compareV(Comparator *v, const ISerializable *s) const = 0;

		//! Tells whether two objects are identical
		/*! \param s Object to compare with
		*/
		bool equals(const ISerializable *s) const; // This method is implemented in comparator.cpp
	};
}

#endif

