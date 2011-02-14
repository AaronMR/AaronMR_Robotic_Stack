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

/*!	\file as_file.cpp
	\brief Binary data reader/writer

	Provides serialization functions for binary file targets.
*/

#include "../include/autoserial/as_file.h"

namespace autoserial
{
	BinaryFileWriter::BinaryFileWriter(const char *fname) : BasicBinaryWriter(false)
	{
		fptr=fopen(fname,"wb");
	}

	Result BinaryFileWriter::flush()
	{
		if(fptr)
		{
			Int32 count;
			for(Size i=0;i<bufferCount;i++)
			{
				count=buffers[i].length;
				if (fwrite(buffers[i].data,count,1,fptr)!=1)
					return AS_FILE_ERROR;
			}
			fflush(fptr);
			return AS_OK;
		}
		return AS_FILE_ERROR;
	}

	BinaryFileWriter::~BinaryFileWriter()
	{
		if(fptr)
			fclose(fptr);
	}

	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////

	Result FlippingBinaryFileWriter::flush()
	{
		if(fptr)
		{
			Int32 count;
			for(Size i=0;i<bufferCount;i++)
			{
				count=buffers[i].length;
				if (fwrite(buffers[i].data,count,1,fptr)!=1)
					return AS_FILE_ERROR;
			}
			return AS_OK;
		}
		return AS_FILE_ERROR;
	}

	FlippingBinaryFileWriter::FlippingBinaryFileWriter(const char *fname) : FlippingBasicBinaryWriter(false)
	{
		fptr=fopen(fname,"wb");
	}

	FlippingBinaryFileWriter::~FlippingBinaryFileWriter()
	{
		if(fptr)
			fclose(fptr);
	}

	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////

	Result BinaryFileReader::readBytes(void *buf, Size len)
	{
		if(fptr)
		{
			if (fread(buf,len,1,fptr)!=1)
				return AS_FILE_ERROR;
			return AS_OK;
		}
		return AS_FILE_ERROR;
	}

	BinaryFileReader::BinaryFileReader(const char *fname) : BasicBinaryReader(NULL)
	{
		fptr=fopen(fname,"rb");
	}

	BinaryFileReader::~BinaryFileReader()
	{
		if(fptr)
			fclose(fptr);
	}

	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////
	
	Result FlippingBinaryFileReader::readBytes(void *buf, Size len)
	{
		if(fptr)
		{
			if (fread(buf,len,1,fptr)!=1)
				return AS_FILE_ERROR;
			return AS_OK;
		}
		return AS_FILE_ERROR;
	}

	FlippingBinaryFileReader::FlippingBinaryFileReader(const char *fname) : FlippingBasicBinaryReader(NULL)
	{
		fptr=fopen(fname,"rb");
	}

	FlippingBinaryFileReader::~FlippingBinaryFileReader()
	{
		if(fptr)
			fclose(fptr);
	}
}
