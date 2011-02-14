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

/*!	\file binaryhandlers.h
	\brief Binary data reader/writer

	Provides basic serialization functions for binary targets.
*/

#ifndef INCLUDED_BINARYHANDLERS_H
#define INCLUDED_BINARYHANDLERS_H

#include "defs.h"
#include "base.h"
#include "buffer.h"

namespace autoserial
{
	//! A list of type names
	/*! This object is exchanged between two peers in order to generate
		indexed types instead of named types for higher performance
		communications.
	*/
	class TypeList : public ISerializable
	{
		AS_CLASSDEF(TypeList)
		AS_MEMBERS
			//! Vector of type names
			AS_ITEM(std::vector<std::string>,typeNames)
		AS_CLASSEND;

	public:
		//! Construct local type list
		Result setupLocalList();
	};

	//! Base class for binary writer with type mapping
	class BasicBinaryWriter : public IOutputStream
	{
	private:
		//! Vector of references
		std::vector<const ISerializable *> refs;

		//! Buffer for copying small items
		internal::Buffer<UInt8> buffer;
		//! Current buffer position
		Size bufferOff;
		//! Start of current block in buffer
		Size bufferStart;

		//! Store types as indexes or strings?
		Bool indexed;

		//! Is the small data buffer currently active?
		Bool bufferActive;

	protected:
		//! Structure to describe large data blocks
		/*! This structure must be identical to WSABUF
		*/
		struct DataBlock
		{
			AS_TYPENAMESTD(DataBlock);

			//! Length of block
			/*! If this value is positive, data is a pointer to the data.
				If this value is negative, data is an offset in buffer.
			*/
			Int32 length;

			union {
				//! Pointer to data
				const void *data;
				Size offset;
			};

			void compare(Comparator *v,const DataBlock& other) const
			{
				/*if(data<0)
					return data==other.data;
				if(length!=other.length)
					return false;
				return ((memcmp(data,other.data,length)==0) ? true : false);*/
				v->enterMember("data");
				if(data<0)
					v->memberEqual(data==other.data);
				else if(length!=other.length)
					v->memberEqual(false);
				else
					v->memberEqual((memcmp(data,other.data,length)==0) ? true : false);
				v->leaveMember("data");
			}
		};

		//! List of buffers
		internal::Buffer<DataBlock> buffers;

		//! Number of buffers
		UInt32 bufferCount;

	private:
		//! Write bytes to target
		Result writeBytes(const void *buf, Size len);

		//! Allocate additional buffers
		Result allocateBuffers();

		//! Reserve space in small data buffer
		/*! \param s Number of bytes to ensure at end of buffer
		*/
		Result reserveBuffer(Size s);

		//! Reset writer
		Result reset();

	protected:
		//! Flush the buffered data to output
		virtual Result flush() = 0;

	public:
		//! Constructor
		/*! \param indexed Initial transfer mode, by name by default
		*/
		BasicBinaryWriter(Bool indexed = false);

		//! Destructor
		virtual ~BasicBinaryWriter() { }

		//! Set transfer mode
		/*! \param indexed Initial transfer mode
			If indexed is set to true the transfer mode is set to type indices,
			otherwise it is by name. Using the indexed transfer mode requires
			the receiver to have the senders type list.
		*/
		Result setTransferMode(Bool indexed) { this->indexed=indexed; return AS_OK; }

		//! Write a reference to target
		virtual Result writeRef(const ISerializable *item);

		//! Write an object to target
		virtual Result write(const ISerializable *item);

		//! Enter named item
		virtual Result enter(const char *name) { return AS_OK; }

		//! Leave named item
		virtual Result leave(const char *name) { return AS_OK; }

		//! Write type name
		virtual Result type(const char *name) { return AS_OK; }

		//! Write signed 8-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeInt8(const Int8& item)		{ return writeBytes(&item,sizeof(Int8)); }

		//! Write unsigned 8-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeUInt8(const UInt8& item)	{ return writeBytes(&item,sizeof(UInt8)); }

		//! Write signed 16-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeInt16(const Int16& item)	{ return writeBytes(&item,sizeof(Int16)); }

		//! Write unsigned 16-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeUInt16(const UInt16& item)	{ return writeBytes(&item,sizeof(UInt16)); }

		//! Write signed 32-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeInt32(const Int32& item)	{ return writeBytes(&item,sizeof(Int32)); }

		//! Write unsigned 32-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeUInt32(const UInt32& item)	{ return writeBytes(&item,sizeof(UInt32)); }

		//! Write signed 64-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeInt64(const Int64& item)	{ return writeBytes(&item,sizeof(Int64)); } 

		//! Write unsigned 64-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeUInt64(const UInt64& item)	{ return writeBytes(&item,sizeof(UInt64)); }

		//! Write 32-bit floating point value
		/*! \param item Value to write
		*/
		virtual Result writeFloat(const Float& item)	{ return writeBytes(&item,sizeof(Float)); }

		//! Write 64-bit floating point value
		/*! \param item Value to write
		*/
		virtual Result writeDouble(const Double& item)	{ return writeBytes(&item,sizeof(Double)); }

		//! Write boolean value
		/*! \param item Value to write
		*/
		virtual Result writeBool(const Bool& item)		
		{ 
			// This is required to please Mac OS X, where gcc makes bool 4 bytes in
			// size, as opposed to almost everybody else who use one byte. We hope
			// the compiler will be intelligent enough to get rid of this constant
			// comparison and compiles this into something efficient.
			if(sizeof(bool)==1)
				return writeBytes(&item,1); 
			else
			{ 
				char u=item?1:0; 
				return writeBytes(&u,1); 
			}
		}

		//! Write an array of simple types
		/*! \param data Address of memory block containing values to write
			\param count Number of elements
			\param size Size of individual elements in bytes
		*/
		virtual Result writeArray(const void *data, const Size count, const Size size) { return writeBytes(data,count*size); }

		//! Write STL string value
		/*! \param item Value to write
		*/
		virtual Result writeString(const std::string& item);

	protected:
		//! Size in bytes of written data
		UInt32 size;

	public:
		//! Get size in bytes of written data
		UInt32 getSize() {return size;};
	};

	//! Base class for binary reader with type mapping
	class BasicBinaryReader : public IInputStream
	{
	private:
		//! Vector of references
		std::vector<ISerializable *> refs;

		//! Type map
		UInt32 *typeMap;

		//! Size of type map
		UInt32 typeMapSize;

	private:
		//! Reset reader
		Result reset();

	protected:
		//! Read bytes from source
		virtual Result readBytes(void *buf, Size len) = 0;

		//! Set a hint for caching
		/*! \param len The number of bytes that will be read in the current process
		*/
		virtual void cacheHint(Size len) { }

	public:
		//! Constructor
		/*! \param otherList List of types of the source
			If the list of types is set, indexed mode is available. By default,
			no type list is selected.
		*/
		BasicBinaryReader(TypeList *otherList = NULL);

		//! Destructor
		virtual ~BasicBinaryReader();

		//! Set source type list
		/*! \param otherList List of types of the source
			When the list of types is set, indexed mode is available.
		*/
		Result setTypeList(TypeList *otherList);

		//! Read a reference from source
		virtual Result readRef(ISerializable **item);

		//! Read an object from source
		/*! \param item Pointer to receive read item
		*/
		virtual Result read(ISerializable **item);

		//! Enter named item
		virtual Result enter(const char *name) { return AS_OK; }

		//! Leave named item
		virtual Result leave(const char *name) { return AS_OK; }

		//! Read signed 8-bit integer
		/*! \param item Value to read
		*/
		virtual Result readInt8(Int8& item)		{ return readBytes(&item,sizeof(Int8)); }

		//! Read unsigned 8-bit integer
		/*! \param item Value to read
		*/
		virtual Result readUInt8(UInt8& item)	{ return readBytes(&item,sizeof(UInt8)); }

		//! Read signed 16-bit integer
		/*! \param item Value to read
		*/
		virtual Result readInt16(Int16& item)	{ return readBytes(&item,sizeof(Int16)); }

		//! Read unsigned 16-bit integer
		/*! \param item Value to read
		*/
		virtual Result readUInt16(UInt16& item)	{ return readBytes(&item,sizeof(UInt16)); }

		//! Read signed 32-bit integer
		/*! \param item Value to read
		*/
		virtual Result readInt32(Int32& item)	{ return readBytes(&item,sizeof(Int32)); }

		//! Read unsigned 32-bit integer
		/*! \param item Value to read
		*/
		virtual Result readUInt32(UInt32& item)	{ return readBytes(&item,sizeof(UInt32)); }

		//! Read signed 64-bit integer
		/*! \param item Value to read
		*/
		virtual Result readInt64(Int64& item)	{ return readBytes(&item,sizeof(Int64)); } 

		//! Read signed 64-bit integer
		/*! \param item Value to read
		*/
		virtual Result readUInt64(UInt64& item)	{ return readBytes(&item,sizeof(UInt64)); }

		//! Read 32-bit floating point value
		/*! \param item Value to read
		*/
		virtual Result readFloat(Float& item)	{ return readBytes(&item,sizeof(Float)); }

		//! Read 64-bit floating point value
		/*! \param item Value to read
		*/
		virtual Result readDouble(Double& item)	{ return readBytes(&item,sizeof(Double)); }

		//! Read boolean value
		/*! \param item Value to read
		*/
		virtual Result readBool(Bool& item)		
		{ 
			// This is required to please Mac OS X, where gcc makes bool 4 bytes in
			// size, as opposed to almost everybody else who use one byte. We hope
			// the compiler will be intelligent enough to get rid of this constant
			// comparison and compiles this into something efficient.
			if(sizeof(bool)==1)
				return readBytes(&item,1);
			else
			{
				char u; 
				Result r=readBytes(&u,1); 
				item=u?true:false;
				return r;
			}
		}

		//! Read an array of simple types
		/*! \param data Address of memory block containing values to read
			\param count Number of elements
			\param size Size of individual elements in bytes
		*/
		virtual Result readArray(void *data, const Size count, const Size size) { return readBytes(data,count*size); }

		//! Read STL string value
		/*! \param item Value to read
		*/
		virtual Result readString(std::string& item);
	};

	//! Contains internal templated functions for byte swapping
	/*!	This class contains multiple specialized function implementations
		for byte swapping. This is used by the serializer when mixed-endian
		machines are communicating together.
	*/
	class ByteSwapper
	{
	public:
		/*! \brief Generic byte swap (internal)
			\ingroup internal
			\param data Pointer to data to byteswap
			Swaps the first size bytes of data.
		*/
		template<int size> static inline void swapp(void *data)
		{
			UInt8 dc[size];
			UInt32 p;
			for(p=0;p<size;p++)
				dc[p]=((UInt8*)data)[p];
			for(p=0;p<size;p++)
				((UInt8*)data)[p]=dc[size-1-p];
		}

		//! Generic byte swap function
		/*! \ingroup internal
			This function should be called whenever bytes have to be swapped.
			It will automagically forward the call to the appropriate internal
			swapping function according to sizeof(item).
			\param item Item to byte-swap.
		*/
		template<typename t> static void swap(t& item)
		{
			swapp<sizeof(t)>(&item);
		}
	};


	//! Swap bytes for 8-bit word (internal)
	/*! \ingroup internal
		\param data Pointer to data to byteswap
		Do nothing, since there is only one byte...
	*/
	template<> inline void ByteSwapper::swapp<1>(void *data)
	{
	}

	//! Swap bytes for 16-bit word (internal)
	/*! \ingroup internal
		\param data Pointer to data to byteswap
		Swaps the first 2 bytes of data.
	*/
	template<> inline void ByteSwapper::swapp<2>(void *data)
	{
		UInt16 a=*(UInt16*)data;
		*(UInt16*)data=
			((a&0x00ff)<< 8)|
			((a>> 8)&0x00ff);
	}
	//! Swap bytes for 32-bit word (internal)
	/*! \ingroup internal
		\param data Pointer to data to byteswap
		Swaps the first 4 bytes of data.
	*/
	template<> inline void ByteSwapper::swapp<4>(void *data)
	{
		UInt32 a=*(UInt32*)data;
		*(UInt32*)data=
			((a&0x000000ff)<<24)|
			((a&0x0000ff00)<< 8)|
			((a>> 8)&0x0000ff00)|
			((a>>24)&0x000000ff);
	}
	//! Swap bytes for 64-bit word (internal)
	/*! \ingroup internal
		\param data Pointer to data to byteswap
		Swaps the first 8 bytes of data.
	*/
	template<> inline void ByteSwapper::swapp<8>(void *data)
	{
		UInt64 a=*(UInt64*)data;
		*(UInt64*)data=
			((a&0x00000000000000ffLL)<<56)|
			((a&0x000000000000ff00LL)<<40)|
			((a&0x0000000000ff0000LL)<<24)|
			((a&0x00000000ff000000LL)<< 8)|
			((a>> 8)&0x00000000ff000000LL)|
			((a>>24)&0x0000000000ff0000LL)|
			((a>>40)&0x000000000000ff00LL)|
			((a>>56)&0x00000000000000ffLL);
	}

	//! Base class for binary reader with type mapping and endian flipping
	/*! This is a variant of BinaryReader that performs endian flipping on all
		elements that are read from the input stream.
	*/
	class FlippingBasicBinaryReader : public BasicBinaryReader
	{
	public:
		//! Constructor
		/*! \param otherList List of types of the source
			If the list of types is set, indexed mode is available. By default,
			no type list is selected.
		*/
		FlippingBasicBinaryReader(TypeList *otherList = NULL) : BasicBinaryReader(otherList) { }

		//! Destructor
		virtual ~FlippingBasicBinaryReader() { }

		//! Read signed 16-bit integer
		/*! \param item Value to read
		*/
		virtual Result readInt16(Int16& item)	{ Result r=readBytes(&item,sizeof(Int16)); ByteSwapper::swap<Int16>(item); return r; }

		//! Read unsigned 16-bit integer
		/*! \param item Value to read
		*/
		virtual Result readUInt16(UInt16& item)	{ Result r=readBytes(&item,sizeof(UInt16)); ByteSwapper::swap<UInt16>(item); return r; }

		//! Read signed 32-bit integer
		/*! \param item Value to read
		*/
		virtual Result readInt32(Int32& item)	{ Result r=readBytes(&item,sizeof(Int32)); ByteSwapper::swap<Int32>(item); return r; }

		//! Read unsigned 32-bit integer
		/*! \param item Value to read
		*/
		virtual Result readUInt32(UInt32& item)	{ Result r=readBytes(&item,sizeof(UInt32)); ByteSwapper::swap<UInt32>(item); return r; }

		//! Read signed 64-bit integer
		/*! \param item Value to read
		*/
		virtual Result readInt64(Int64& item)	{ Result r=readBytes(&item,sizeof(Int64)); ByteSwapper::swap<Int64>(item); return r; } 

		//! Read signed 64-bit integer
		/*! \param item Value to read
		*/
		virtual Result readUInt64(UInt64& item)	{ Result r=readBytes(&item,sizeof(UInt64)); ByteSwapper::swap<UInt64>(item); return r; }

		//! Read 32-bit floating point value
		/*! \param item Value to read
		*/
		virtual Result readFloat(Float& item)	{ Result r=readBytes(&item,sizeof(Float)); ByteSwapper::swap<Float>(item); return r; }

		//! Read 64-bit floating point value
		/*! \param item Value to read
		*/
		virtual Result readDouble(Double& item)	{ Result r=readBytes(&item,sizeof(Double)); ByteSwapper::swap<Double>(item); return r; }

		//! Read an array of simple types
		/*! \param data Address of memory block containing values to read
			\param count Number of elements
			\param size Size of individual elements in bytes
		*/
		virtual Result readArray(void *data, const Size count, const Size size);
	};


	//! Base class for binary writer with type mapping
	/*! This is a variant of BinaryWriter that performs endian flipping on all
		elements that are written to the output stream.
	*/
	class FlippingBasicBinaryWriter : public IOutputStream
	{
	private:
		//! Vector of references
		std::vector<const ISerializable *> refs;

		//! Buffer for copying small items
		internal::Buffer<UInt8> buffer;
		//! Current buffer position
		Size bufferOff;
		//! Start of current block in buffer
		Size bufferStart;

		//! Store types as indexes or strings?
		Bool indexed;

		//! Is the small data buffer currently active?
		Bool bufferActive;

	protected:
		//! Structure to describe large data blocks
		/*! This structure must be identical to WSABUF
		*/
		struct DataBlock
		{
			AS_TYPENAMESTD(DataBlock);

			//! Length of block
			/*! If this value is positive, data is a pointer to the data.
				If this value is negative, data is an offset in buffer.
			*/
			Int32 length;

			union {
				//! Pointer to data
				const void *data;
				Size offset;
			};

			void compare(Comparator *v,const DataBlock& other) const
			{
				/*if(data<0)
					return data==other.data;
				if(length!=other.length)
					return false;
				return ((memcmp(data,other.data,length)==0) ? true : false);*/
				v->enterMember("data");
				if(data<0)
					v->memberEqual(data==other.data);
				else if(length!=other.length)
					v->memberEqual(false);
				else
					v->memberEqual((memcmp(data,other.data,length)==0) ? true : false);
				v->leaveMember("data");
			}
		};

		//! List of buffers
		internal::Buffer<DataBlock> buffers;

		//! Number of buffers
		UInt32 bufferCount;

	private:
		//! Write bytes to target
		Result writeBytes(const void *buf, Size len);

		//! Allocate additional buffers
		Result allocateBuffers();

		//! Reserve space in small data buffer
		/*! \param s Number of bytes to ensure at end of buffer
		*/
		Result reserveBuffer(Size s);

		//! Reset writer
		Result reset();

	protected:
		//! Flush the buffered data to output
		virtual Result flush() = 0;

	public:
		//! Constructor
		/*! \param indexed Initial transfer mode, by name by default
		*/
		FlippingBasicBinaryWriter(Bool indexed = false);

		//! Destructor
		virtual ~FlippingBasicBinaryWriter() { }

		//! Set transfer mode
		/*! \param indexed Initial transfer mode
			If indexed is set to true the transfer mode is set to type indices,
			otherwise it is by name. Using the indexed transfer mode requires
			the receiver to have the senders type list.
		*/
		Result setTransferMode(Bool indexed) { this->indexed=indexed; return AS_OK; }

		//! Write a reference to target
		virtual Result writeRef(const ISerializable *item);

		//! Write an object to target
		virtual Result write(const ISerializable *item);

		//! Enter named item
		virtual Result enter(const char *name) { return AS_OK; }

		//! Leave named item
		virtual Result leave(const char *name) { return AS_OK; }

		//! Write type name
		virtual Result type(const char *name) { return AS_OK; }

		//! Write signed 8-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeInt8(const Int8& item)		{ Int8 it=item; ByteSwapper::swap<Int8>(it); return writeBytes(&it,sizeof(Int8)); }

		//! Write unsigned 8-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeUInt8(const UInt8& item)	{ UInt8 it=item; ByteSwapper::swap<UInt8>(it);return writeBytes(&it,sizeof(UInt8)); }

		//! Write signed 16-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeInt16(const Int16& item)	{ Int16 it=item; ByteSwapper::swap<Int16>(it);return writeBytes(&it,sizeof(Int16)); }

		//! Write unsigned 16-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeUInt16(const UInt16& item)	{ UInt16 it=item; ByteSwapper::swap<UInt16>(it);return writeBytes(&it,sizeof(UInt16)); }

		//! Write signed 32-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeInt32(const Int32& item)	{ Int32 it=item; ByteSwapper::swap<Int32>(it);return writeBytes(&it,sizeof(Int32)); }

		//! Write unsigned 32-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeUInt32(const UInt32& item)	{ UInt32 it=item; ByteSwapper::swap<UInt32>(it);return writeBytes(&it,sizeof(UInt32)); }

		//! Write signed 64-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeInt64(const Int64& item)	{ Int64 it=item; ByteSwapper::swap<Int64>(it);return writeBytes(&it,sizeof(Int64)); } 

		//! Write unsigned 64-bit integer
		/*! \param item Value to write
		*/
		virtual Result writeUInt64(const UInt64& item)	{ UInt64 it=item; ByteSwapper::swap<UInt64>(it);return writeBytes(&it,sizeof(UInt64)); }

		//! Write 32-bit floating point value
		/*! \param item Value to write
		*/
		virtual Result writeFloat(const Float& item)	{ Float it=item; ByteSwapper::swap<Float>(it);return writeBytes(&it,sizeof(Float)); }

		//! Write 64-bit floating point value
		/*! \param item Value to write
		*/
		virtual Result writeDouble(const Double& item)	{ Double it=item; ByteSwapper::swap<Double>(it);return writeBytes(&it,sizeof(Double)); }

		//! Write boolean value
		/*! \param item Value to write
		*/
		virtual Result writeBool(const Bool& item)		
		{ 
			// This is required to please Mac OS X, where gcc makes bool 4 bytes in
			// size, as opposed to almost everybody else who use one byte. We hope
			// the compiler will be intelligent enough to get rid of this constant
			// comparison and compiles this into something efficient.
			if(sizeof(bool)==1)
				return writeBytes(&item,1); 
			else
			{ 
				char u=item?1:0; 
				return writeBytes(&u,1); 
			}
		}

		//! Write an array of simple types
		/*! \param data Address of memory block containing values to write
			\param count Number of elements
			\param size Size of individual elements in bytes
		*/
		virtual Result writeArray(const void *data, const Size count, const Size size);

		//! Write STL string value
		/*! \param item Value to write
		*/
		virtual Result writeString(const std::string& item);

	protected:

		//! Size in bytes of written data
		UInt32 size;

	public:
		//! Get size in bytes of written data
		UInt32 getSize() {return size;};
	};
}
#endif

