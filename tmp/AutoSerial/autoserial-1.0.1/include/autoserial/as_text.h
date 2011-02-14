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

/*!	\file as_text.h
	\brief Text data reader/writer

*/

#ifndef INCLUDED_AS_TEXT_H
#define INCLUDED_AS_TEXT_H

#include "base.h"
#include "buffer.h"

namespace autoserial
{
	//! Base class for Text writer with type mapping
	class BasicTextWriter : public IOutputStream
	{
	private:
		//! Vector of references
		std::vector<const ISerializable *> refs;

	protected:
		//! Output stream for text
		std::ostream& out;

	private:
		//! Reset writer
		Result reset();

	protected:
		//! Flush the buffered data to output
		virtual Result flush() = 0;

	public:
		//! Constructor
		/*! \param o Output stream to write to
		*/
		BasicTextWriter(std::ostream& o) : out(o) { }

		//! Destructor
		virtual ~BasicTextWriter() { }

		//! Write a reference to target
		virtual Result writeRef(const ISerializable *item);

		//! Write an object to target
		virtual Result write(const ISerializable *item);

		//! Enter named item
		virtual Result enter(const char *name) { return AS_OK; }

		//! Leave named item
		virtual Result leave(const char *name) { return AS_OK; }

		virtual Result writeInt8(const Int8& item)		{ out << item; return AS_OK; }
		virtual Result writeUInt8(const UInt8& item)	{ out << item; return AS_OK; }
		virtual Result writeInt16(const Int16& item)	{ out << item; return AS_OK;  }
		virtual Result writeUInt16(const UInt16& item)	{ out << item; return AS_OK;  }
		virtual Result writeInt32(const Int32& item)	{ out << item; return AS_OK;  }
		virtual Result writeUInt32(const UInt32& item)	{ out << item; return AS_OK;  }
		virtual Result writeInt64(const Int64& item)	{ out << item; return AS_OK;  }
		virtual Result writeUInt64(const UInt64& item)	{ out << item; return AS_OK;  }
		virtual Result writeFloat(const Float& item)	{ out << item; return AS_OK;  }
		virtual Result writeDouble(const Double& item)	{ out << item; return AS_OK;  }
		virtual Result writeBool(const Bool& item)		{ out << item; return AS_OK;  }
		virtual Result writeArray(const void *data, const Size count, const Size size);
		virtual Result writeString(const std::string& item) { out << item; return AS_OK; }
	};

	//! Base class for Text reader with type mapping
	class BasicTextReader : public IInputStream
	{
	private:
		//! Vector of references
		std::vector<ISerializable *> refs;

	private:
		//! Reset reader
		Result reset();

	protected:
		//! Read bytes from source
		virtual Result readBytes(void *buf, Size len) = 0;

	public:
		//! Constructor
		BasicTextReader() { }

		//! Destructor
		virtual ~BasicTextReader() { }

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

		virtual Result readInt8(Int8& item)		{ return readBytes(&item,sizeof(Int8)); }
		virtual Result readUInt8(UInt8& item)	{ return readBytes(&item,sizeof(UInt8)); }
		virtual Result readInt16(Int16& item)	{ return readBytes(&item,sizeof(Int16)); }
		virtual Result readUInt16(UInt16& item)	{ return readBytes(&item,sizeof(UInt16)); }
		virtual Result readInt32(Int32& item)	{ return readBytes(&item,sizeof(Int32)); }
		virtual Result readUInt32(UInt32& item)	{ return readBytes(&item,sizeof(UInt32)); }
		virtual Result readInt64(Int64& item)	{ return readBytes(&item,sizeof(Int64)); } 
		virtual Result readUInt64(UInt64& item)	{ return readBytes(&item,sizeof(UInt64)); }
		virtual Result readFloat(Float& item)	{ return readBytes(&item,sizeof(Float)); }
		virtual Result readDouble(Double& item)	{ return readBytes(&item,sizeof(Double)); }
		virtual Result readBool(Bool& item)		{ return readBytes(&item,1); }
		virtual Result readArray(void *data, const Size count, const Size size) { return readBytes(data,count*size); }
		virtual Result readString(std::string& item);
	};

	//! Specialized TextWriter for XML files
	class XMLFileWriter : public BasicTextWriter
	{
	private:
		//! File handle to write to
		std::ofstream file;

	protected:
		//! Flush the buffered data to output
		virtual Result flush();

		//! Enter named item
		virtual Result enter(const char *name) { file << "<" << name << ">"; return AS_OK; }

		//! Leave named item
		virtual Result leave(const char *name) { file << "</" << name << ">" << std::endl; return AS_OK; }

		//! Write down type name
		virtual Result type(const char *name) { return AS_OK; }

	public:
		//! Constructor
		XMLFileWriter(const char *fname);

		//! Destructor
		virtual ~XMLFileWriter();

		//! Close output stream
		//void close() { file.close(); }
	};

#if 0
	//! Specialized TextReader for files
	class TextFileReader : public BasicTextReader
	{
	private:
		//! File handle to write to
		FILE *fptr;

	protected:
		//! Flush the buffered data to output
		virtual Result readBytes(void *buf, Size len);

	public:
		//! Constructor
		TextFileReader(const char *fname);

		//! Destructor
		~TextFileReader();
	};
#endif


		//! Handlers for textual serialization
	class DebuggerWriter :public BasicTextWriter
	{
	private:
		int count;
		std::stringstream stream;

	protected:
		//! Flush the buffered data to output
		virtual Result flush();

		//! Enter named item
		virtual Result enter(const char *name) { stream << "{" << name << ";"; return AS_OK; }

		//! Leave named item
		virtual Result leave(const char *name) { stream  << "};"; return AS_OK; }

		//! Write down type name
		virtual Result type(const char *name) { stream << "#TYPE#" << name << ";"; return AS_OK; }

	public:
		//! Constructor
		DebuggerWriter()  : BasicTextWriter(stream) { count = 0; }

		//! Destructor
		virtual ~DebuggerWriter() { }

		virtual Result writeInt8(const Int8& item)		{ stream  << item << ";"; return AS_OK; }
		virtual Result writeUInt8(const UInt8& item)	{ stream << item << ";"; return AS_OK; }
		virtual Result writeInt16(const Int16& item)	{ stream  << item << ";"; return AS_OK;  }
		virtual Result writeUInt16(const UInt16& item)	{ stream << item << ";"; return AS_OK;  }
		virtual Result writeInt32(const Int32& item)	{ stream  << item << ";"; return AS_OK;  }
		virtual Result writeUInt32(const UInt32& item)	{ stream << item << ";"; return AS_OK;  }
		virtual Result writeInt64(const Int64& item)	{ stream << item << ";"; return AS_OK;  }
		virtual Result writeUInt64(const UInt64& item)	{ stream  << item << ";"; return AS_OK;  }
		virtual Result writeFloat(const Float& item)	{ stream  << item << ";"; return AS_OK;  }
		virtual Result writeDouble(const Double& item)	{ stream  << item << ";"; return AS_OK;  }
		virtual Result writeBool(const Bool& item)		{ stream  << item << ";"; return AS_OK;  }
		virtual Result writeArray(const void *data, const Size count, const Size size);
		virtual Result writeString(const std::string& item) { stream << item << ";"; return AS_OK; }

		Int32 getCount() { return count; }

		Result reset() { stream.str(""); count++; return AS_OK; }

		std::stringstream& getStream() { return stream; }
	};

	//! Handlers for textual deserialization
	class DebuggerReader : public BasicTextReader
	{
	private:
		int i;
		std::ofstream infile;
		char* buffer;

	protected:
		//! Flush the buffered data to output
		virtual Result flush() {return AS_OK;}

		//! Enter named item
		virtual Result enter(const char *name) { file << "{" << name << " -"; return AS_OK; }

		//! Leave named item
		virtual Result leave(const char *name) { file  << "};"; return AS_OK; }

		virtual Result type (const char *name) { file << "#TYPE#" << name << ";"; return AS_OK; }

		virtual Result readBytes(void *buf, Size len);


	public:
		std::ostringstream file;

		//! Constructor
		DebuggerReader(char* buf) {
			this->buffer = buf;
		}

		//! Destructor
		virtual ~DebuggerReader() {}

		/*Result dispatch(const char* cmd,const ISerializable *item) {
		i++;
		return ret;
		}*/

		/*virtual Result writeRef(const ISerializable *item) {
		item->
		file << item->getTypeNameV() << ";";// << std::endl;
		return AS_OK;
		}*/

		virtual Result readInt8(const Int8& item)	{ std::cout  << "HELLO" << item << ";"; return AS_OK; }
		virtual Result readUInt8(const UInt8& item)	{ file << item << ";"; return AS_OK; }
		virtual Result readInt16(const Int16& item)	{ file  << item << ";"; return AS_OK;  }
		virtual Result readUInt16(const UInt16& item)	{ file << item << ";"; return AS_OK;  }
		virtual Result readInt32(const Int32& item)	{ file  << item << ";"; return AS_OK;  }
		virtual Result readUInt32(const UInt32& item)	{ file << item << ";"; return AS_OK;  }
		virtual Result readInt64(const Int64& item)	{ file << item << ";"; return AS_OK;  }
		virtual Result readUInt64(const UInt64& item)	{ file  << item << ";"; return AS_OK;  }
		virtual Result readFloat(const Float& item)	{ file  << item << ";"; return AS_OK;  }
		virtual Result readDouble(const Double& item)	{ file  << item << ";"; return AS_OK;  }
		virtual Result readBool(const Bool& item)	{ file  << item << ";"; return AS_OK;  }
		virtual Result readArray(const void *data, const Size count, const Size size) {return AS_OK;}
		virtual Result readString(const std::string& item) { file << item << ";"; return AS_OK; }

		Int32 getCount() { return i; }

		//std::ostringstream getStream() { return file; }

		Result reset() { this->file.str(""); i++; return AS_OK; }
	};
}

#endif
