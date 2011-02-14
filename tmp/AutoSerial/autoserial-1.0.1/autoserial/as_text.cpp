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

/*!	\file as_text.cpp
	\brief Text data reader/writer

*/

#include "../include/autoserial/as_text.h"
#include "../include/autoserial/classfactory.h"

namespace autoserial
{
	Result BasicTextWriter::reset()
	{
		refs.clear();
		refs.push_back(NULL);
		return AS_OK;
	}

	Result BasicTextWriter::writeRef(const ISerializable *item)
	{
		for(Size i=0;i<refs.size();i++)
		{
			if(refs[i]==item)
			{
				XPSize s=(XPSize)i;
				return typehandler::write(this,s);
			}
		}
		refs.push_back(item);
		XPSize s=(XPSize)refs.size()-1;
		return typehandler::write(this,s);
	}

	Result BasicTextWriter::write(const ISerializable *item)
	{
		assert(item!=NULL);

		reset();

		//DLOG((textLog.write(1) << "Writing " << item->getTypeNameV()));

		refs.push_back(item);
		for(Size i=1;i<refs.size();i++)
		{
			enter("item");
			//DLOG((textLog.write(2) << "Reference " << ((int)i) << " is " << refs[i]->getTypeNameV()));

			Result r=refs[i]->writeV(this);
			if(AS_FAILED(r))
				return r;
			leave("item");
		}
		enter("refs");
		for(Size i=1;i<refs.size();i++)
		{
			enter("ref");
			writeString(std::string(refs[i]->getTypeNameV()));
			leave("ref");
		}
		leave("refs");

		return flush();
	}

	Result BasicTextWriter::writeArray(const void *data, const Size count, const Size size)
	{
		Size ucount=count;
/*		if(Writer::ShortForm && count>1)
			ucount=1;
*/		UInt8 *d=(UInt8*)data;
		for(Size i=0;i<ucount;i++)
		{
			if(i>0) out << ",";
			switch(size)
			{
			case 1: out << (int)(*(UInt8*)d); break;
			case 2: out << (*(UInt16*)d); break;
			case 4: out << (*(UInt32*)d); break;
			case 8: out << (*(UInt64*)d); break;
			}
			d+=size;
		}
		//return true;
		return AS_OK;
	}

	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////

	Result BasicTextReader::reset()
	{
		refs.clear();
		return AS_OK;
	}

	Result BasicTextReader::readRef(ISerializable **item)
	{
		XPSize ndx;
		Result r;
		if(AS_FAILED(r=typehandler::read(this,ndx)))
			return r;
		assert(ndx>=0 && ndx<refs.size());
		*item=refs[ndx];
		return AS_OK;
	}

	Result BasicTextReader::read(ISerializable **item)
	{
		reset();

		UInt32 ts, ds;
		Result r;
		
		if(AS_FAILED(r=readBytes(&ts,4)))
			return r;
		if(AS_FAILED(r=readBytes(&ds,4)))
			return r;
		UInt32 *types=(UInt32*)alloca(ds);
		if(AS_FAILED(r=readBytes(types,ds)))
			return r;

		refs.push_back(NULL);
		char *d=(char*)types;
		for(Size i=0;i<ds;i+=strlen(&d[i])+1)
		{
			ISerializable *ni;
			ni=(ISerializable *)internal::ClassFactory::create(&d[i] CREATEARGS);
			if(ni==NULL)
			{
				for(Size i=1;i<refs.size();i++)
					delete refs[i];
				return AS_FAIL;
			}
			refs.push_back(ni);
		}

		for(Size i=1;i<refs.size();i++)
		{
			if(AS_FAILED(refs[i]->readV(this)))
			{
				for(Size i=1;i<refs.size();i++)
					delete refs[i];
				return AS_FAIL;
			}
		}
		*item=refs[1];
		return AS_OK;
	}

	Result BasicTextReader::readString(std::string& item)
	{
		XPSize s;
		Result r;
		if(AS_FAILED(r=typehandler::read(this,s)))
			return r;
		assert(s<1048576);
		char *tmp=(char *)alloca(s+1);
		if(AS_FAILED(r=readBytes(tmp,s)))
			return r;
		tmp[s]=0;
		item=tmp;
		return AS_OK;
	}

	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////

	Result XMLFileWriter::flush()
	{
		leave("xmlserializeddata");

		return AS_OK;
	}

	XMLFileWriter::XMLFileWriter(const char *fname) : BasicTextWriter(file)
	{
		file.open(fname,std::ios_base::out|std::ios_base::trunc);
		
		file << "<?xml version=\"1.0\" encoding=\"utf-8\"?>";

		enter("xmlserializeddata");
	}

	XMLFileWriter::~XMLFileWriter()
	{
		file.close();
	}


	Result DebuggerWriter::flush()
	{
		//leave("testSerializedata");
		return AS_OK;
	}

	Result DebuggerWriter::writeArray(const void *data, const Size count, const Size size)
	{		
		Size ucount= 0;
		if (count > 6)
			ucount = 6;
		else 
			ucount = count;

		UInt8 *d=(UInt8*)data;
		for(Size i=0; i<ucount ;i++)
		{
			if(i>0) stream << ",";
			switch(size)
			{
			case 1: stream << (int)(*(UInt8*)d); break;
			case 2: stream << (*(UInt16*)d); break;
			case 4: stream << (*(UInt32*)d); break;
			case 8: stream << (*(UInt64*)d); break;
			}
			d+=size;
		}

		stream << ";";
		return AS_OK; 
	}

	Result DebuggerReader::readBytes(void *buf, Size len)
	{
		memcpy(buf, buffer, len);
		return AS_OK;
	}
}
