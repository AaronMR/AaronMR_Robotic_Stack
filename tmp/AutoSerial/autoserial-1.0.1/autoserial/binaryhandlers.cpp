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

/*!	\file binaryhandlers.cpp
	\brief Binary data reader/writer

*/

#include "../include/autoserial/defs.h"
#include "../include/autoserial/binaryhandlers.h"


namespace autoserial
{
	Result TypeList::setupLocalList()
	{
		for(Size j=0;j<internal::ClassFactory::length();j++)
			typeNames.push_back(std::string(internal::ClassFactory::getName(j)));
		return AS_OK;
	}

	BasicBinaryWriter::BasicBinaryWriter(Bool indexed)
	{ 
		this->indexed=indexed;
		buffer.resize(32768);
		buffers.resize(2048);
		size=0;
	}

	Result BasicBinaryWriter::reset()
	{
		bufferOff=0;
		bufferCount=1;
		bufferActive=false;
		refs.clear();
		refs.push_back(NULL);
		return AS_OK;
	}

	Result BasicBinaryWriter::allocateBuffers()
	{
		if(bufferCount>=buffers.size()-1)
			buffers.resize(buffers.size()*2);
		return AS_OK;
	}

	Result BasicBinaryWriter::reserveBuffer(Size len)
	{
		if(bufferOff+len>buffer.size())
		{
			Size ns=buffer.size()*2;
			while(bufferOff+len>ns)
				ns*=2;
			buffer.resize(ns);
		}
		return AS_OK;
	}


	Result BasicBinaryWriter::writeBytes(const void *buf, Size len) 
	{
		size+=(UInt32)len;

		if(len<16)
		{
			if(!bufferActive)
			{
				bufferStart=bufferOff;
				bufferActive=true;
			}
			reserveBuffer(len);
			memcpy(&buffer[bufferOff],buf,len);
			bufferOff+=len;
		}
		else
		{
			if(bufferActive)
			{
				allocateBuffers();
				buffers[bufferCount].offset=bufferStart;
				buffers[bufferCount].length=-(Int32)(bufferOff-bufferStart);
				bufferCount++;
				bufferActive=false;
			}
			allocateBuffers();
			buffers[bufferCount].data=buf;
			buffers[bufferCount].length=(UInt32)len;
			bufferCount++;
		}
		return AS_OK; 
	}

	Result BasicBinaryWriter::writeRef(const ISerializable *item)
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

	Result BasicBinaryWriter::write(const ISerializable *item)
	{
		assert(item!=NULL);

		reset();

		refs.push_back(item);
		for(Size i=1;i<refs.size();i++)
		{
			Result r=refs[i]->writeV(this);
			if(AS_FAILED(r))
			{
				//std::cerr << "Failed write of " << refs[i]->getTypeNameV();
				return r;
			}
		}

		if(bufferActive)
		{
			allocateBuffers();
			buffers[bufferCount].offset=bufferStart;
			buffers[bufferCount].length=-(Int32)(bufferOff-bufferStart);
			bufferCount++;
			bufferActive=false;
		}
		bufferOff=(bufferOff+3)&(~3);
		bufferStart=bufferOff;

		Size asize=12;
		if(indexed)
			asize+=4*(refs.size()-1);
		else
		{
			for(Size i=1;i<refs.size();i++)
				asize+=strlen(refs[i]->getTypeNameV())+1;
		}

		reserveBuffer(asize);
		UInt32 *h=(UInt32*)&buffer[bufferStart];

		buffers[0].data=h;
		buffers[0].length=(Int32)asize;

		Size totalSize=0;
		for(Size i=0;i<bufferCount;i++)
			totalSize+=abs(buffers[i].length);
		h[0]=(UInt32)(totalSize-8);
		h[1]=indexed ? 0x12345678 : 0x12345679;
		h[2]=buffers[0].length-12;

		if(indexed)
		{
			Int32 ho=3;
			for(Size i=1;i<refs.size();i++)
				h[ho++]=(UInt32)refs[i]->getFactoryTypeIndexV();
		}
		else
		{
			char *d=(char*)&h[3];
			for(Size i=1;i<refs.size();i++)
			{
				strcpy(d,refs[i]->getTypeNameV());
				d+=1+strlen(refs[i]->getTypeNameV());
			}
		}

		// Convert all buffer offsets into pointers
		for(Size i=0;i<bufferCount;++i)
		{
			if(buffers[i].length<0)
			{
				buffers[i].data=&buffer[buffers[i].offset];
				buffers[i].length=-buffers[i].length;
			}
		}

		return flush();
	}

	Result BasicBinaryWriter::writeString(const std::string& item)
	{
		XPSize s=(XPSize)item.length();
		Result r;
		if(AS_FAILED(r=typehandler::write(this,s)))
			return r;
		return writeBytes(item.c_str(),s);
	}

	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////

	Result BasicBinaryReader::reset()
	{
		refs.clear();
		return AS_OK;
	}

	BasicBinaryReader::BasicBinaryReader(TypeList *otherList)
	{
		typeMap=NULL;
		typeMapSize=0;
		if(otherList)
			setTypeList(otherList);
	}

	BasicBinaryReader::~BasicBinaryReader()
	{
		if(typeMap)
			delete typeMap;
	}

	Result BasicBinaryReader::setTypeList(TypeList *otherList)
	{
		if(typeMap)
			delete typeMap;
		typeMap=NULL;
		typeMapSize=0;
		if(otherList)
		{
			typeMapSize=(UInt32)otherList->typeNames.size();
			typeMap=new UInt32[typeMapSize];
			for(Size i=0;i<typeMapSize;i++)
			{
				typeMap[i]=(UInt32)-1;
				for(Size j=0;j<internal::ClassFactory::length();j++)
				{
					if(!strcmp(internal::ClassFactory::getName(j),otherList->typeNames[i].c_str()))
						typeMap[i]=(UInt32)j;
				}
			}
		}
		return AS_OK;
	}

	Result BasicBinaryReader::readRef(ISerializable **item)
	{
		XPSize ndx;
		Result r;
		if(AS_FAILED(r=typehandler::read(this,ndx)))
			return r;
		assert(ndx>=0 && ndx<refs.size());
		*item=refs[ndx];
		return AS_OK;
	}

	Result BasicBinaryReader::read(ISerializable **item)
	{
		reset();

		UInt32 ts, magic, ds;
		Result r;
		bool flip=false;
		
		if(AS_FAILED(r=readUInt32(ts))) return r;

		// Set cache hint
		cacheHint(ts+4);

		// Don't flip magic
		if(AS_FAILED(r=readBytes(&magic,4))) return r;
		if(magic!=0x12345678 && magic!=0x12345679)
		{
			if(magic!=0x78563412 && magic!=0x79563412)
				return AS_FAIL;
			flip=true;
		}
		if(AS_FAILED(r=readUInt32(ds))) return r;
		UInt32 *types=(UInt32*)alloca(ds);
		if(AS_FAILED(r=readBytes(types,ds))) return r;

		refs.push_back(NULL);
		if(magic==0x12345678 || magic==0x78563412)
		{
			if(typeMap==0)
			{
				//binLog.write(0) << "Type list not set!";
				return AS_FAIL;
			}
			for(Size i=0;i<ds/4;i++)
			{
				ISerializable *ni;
				if(flip)
					ByteSwapper::swap<UInt32>(types[i]);
				if(types[i]>=typeMapSize)
				{
					// binLog.write(0) << "Invalid type index, out of range "<< ((UInt32)i);
					for(Size i=1;i<refs.size();i++)
						delete refs[i];
					return AS_FAIL;
				}
				UInt32 t=typeMap[types[i]];
				if(t==(UInt32)-1)
				{
					//binLog.write(0) << "Invalid type construction request";
					for(Size i=1;i<refs.size();i++)
						delete refs[i];
					return AS_FAIL;
				}
				ni=(ISerializable *)internal::ClassFactory::create(t CREATEARGS);
				if(ni==NULL)
				{
					for(Size i=1;i<refs.size();i++)
						delete refs[i];
					return AS_FAIL;
				}
				refs.push_back(ni);
			}
		}
		else
		{
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
		}

		//DLOG((binLog.write(3) << "Reading " << refs[1]->getTypeNameV() << (magic==0x12345678 ? " with indexed types" : " with string types")));

		for(Size i=1;i<refs.size();i++)
		{
			//DLOG((binLog.write(4) << "Reference " << ((int)i) << " is " << refs[i]->getTypeNameV()));
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

	Result BasicBinaryReader::readString(std::string& item)
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

	Result FlippingBasicBinaryReader::readArray(void *data, const Size count, const Size size)
	{
		Result r=readBytes(data,count*size);
		unsigned char *d=(unsigned char *)data;
		switch(size)
		{
		case 2:	for(Size i=0;i<count;++i) ByteSwapper::swapp<2>(&d[2*i]); break;
		case 4:	for(Size i=0;i<count;++i) ByteSwapper::swapp<4>(&d[4*i]); break;
		case 8:	for(Size i=0;i<count;++i) ByteSwapper::swapp<8>(&d[8*i]); break;
		}
		return r;
	}


	/////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////

	FlippingBasicBinaryWriter::FlippingBasicBinaryWriter(Bool indexed)
	{ 
		this->indexed=indexed;
		buffer.resize(32768);
		buffers.resize(2048);
		size=0;
	}

	Result FlippingBasicBinaryWriter::reset()
	{
		bufferOff=0;
		bufferCount=1;
		bufferActive=false;
		refs.clear();
		refs.push_back(NULL);
		return AS_OK;
	}

	Result FlippingBasicBinaryWriter::allocateBuffers()
	{
		if(bufferCount>=buffers.size()-1)
			buffers.resize(buffers.size()*2);
		return AS_OK;
	}

	Result FlippingBasicBinaryWriter::reserveBuffer(Size len)
	{
		if(bufferOff+len>buffer.size())
		{
			Size ns=buffer.size()*2;
			while(bufferOff+len>ns)
				ns*=2;
			buffer.resize(ns);
		}
		return AS_OK;
	}


	Result FlippingBasicBinaryWriter::writeBytes(const void *buf, Size len) 
	{
		size+=(UInt32)len;
		if(len<16)
		{
			if(!bufferActive)
			{
				bufferStart=bufferOff;
				bufferActive=true;
			}
			reserveBuffer(len);
			memcpy(&buffer[bufferOff],buf,len);
			bufferOff+=len;
		}
		else
		{
			if(bufferActive)
			{
				allocateBuffers();
				buffers[bufferCount].offset=bufferStart;
				buffers[bufferCount].length=-(Int32)(bufferOff-bufferStart);
				bufferCount++;
				bufferActive=false;
			}
			allocateBuffers();
			buffers[bufferCount].data=buf;
			buffers[bufferCount].length=(UInt32)len;
			bufferCount++;
		}
		return AS_OK; 
	}

	Result FlippingBasicBinaryWriter::writeRef(const ISerializable *item)
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

	Result FlippingBasicBinaryWriter::write(const ISerializable *item)
	{
		assert(item!=NULL);

		reset();

		//DLOG((binLog.write(3) << "Writing " << item->getTypeNameV() << (indexed ? " with indexed types" : " with string types")));


		refs.push_back(item);
		for(Size i=1;i<refs.size();i++)
		{
			//DLOG((binLog.write(4) << "Reference " << ((int)i) << " is " << refs[i]->getTypeNameV() << " [" << ((UInt32)refs[i]->getFactoryTypeIndexV()) << "]"));

			Result r=refs[i]->writeV(this);
			if(AS_FAILED(r))
			{
				//DLOG((binLog.write(1) << "Failed write of " << refs[i]->getTypeNameV()));
				return r;
			}
		}

		if(bufferActive)
		{
			allocateBuffers();
			buffers[bufferCount].offset=bufferStart;
			buffers[bufferCount].length=-(Int32)(bufferOff-bufferStart);
			bufferCount++;
			bufferActive=false;
		}
		bufferOff=(bufferOff+3)&(~3);
		bufferStart=bufferOff;

		Size asize=12;
		if(indexed)
			asize+=4*(refs.size()-1);
		else
		{
			for(Size i=1;i<refs.size();i++)
				asize+=strlen(refs[i]->getTypeNameV())+1;
		}

		reserveBuffer(asize);
		UInt32 *h=(UInt32*)&buffer[bufferStart];

		buffers[0].data=h;
		buffers[0].length=(Int32)asize;

		Size totalSize=0;
		for(Size i=0;i<bufferCount;i++)
			totalSize+=abs(buffers[i].length);
		
		h[0]=(UInt32)(totalSize-8);	
		ByteSwapper::swap<UInt32>(h[0]);
		h[1]=indexed ? 0x12345678 : 0x12345679; 
		ByteSwapper::swap<UInt32>(h[1]);
		h[2]=buffers[0].length-12;    
		ByteSwapper::swap<UInt32>(h[2]);

		if(indexed)
		{
			Int32 ho=3;
			for(Size i=1;i<refs.size();i++)
				h[ho++]=(UInt32)refs[i]->getFactoryTypeIndexV();
		}
		else
		{
			char *d=(char*)&h[3];
			for(Size i=1;i<refs.size();i++)
			{
				strcpy(d,refs[i]->getTypeNameV());
				d+=1+strlen(refs[i]->getTypeNameV());
			}
		}

		// Convert all buffer offsets into pointers
		for(Size i=0;i<bufferCount;++i)
		{
			if(buffers[i].length<0)
			{
				buffers[i].data=&buffer[buffers[i].offset];
				buffers[i].length=-buffers[i].length;
			}
		}

		return flush();
	}

	Result FlippingBasicBinaryWriter::writeArray(const void *data, const Size count, const Size size){
		unsigned char *d=(unsigned char *)data;
		switch(size)
		{
		case 2:	for(Size i=0;i<count;++i) ByteSwapper::swapp<2>(&d[2*i]); break;
		case 4:	for(Size i=0;i<count;++i) ByteSwapper::swapp<4>(&d[4*i]); break;
		case 8:	for(Size i=0;i<count;++i) ByteSwapper::swapp<8>(&d[8*i]); break;
		}
		return writeBytes(data,count*size); 
	}

	Result FlippingBasicBinaryWriter::writeString(const std::string& item)
	{
		XPSize s=(XPSize)item.length();
		Result r;
		if(AS_FAILED(r=typehandler::write(this,s)))
			return r;
		return writeBytes(item.c_str(),s);
	}
}
