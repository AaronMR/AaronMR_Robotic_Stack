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

#ifndef OBJECTS_INCLUDED_H
#define OBJECTS_INCLUDED_H

#include <autoserial/autoserial.h>
#include <vector>
//#include <stdio.h>

// Define various types of serializable objects

class IntDouble : public autoserial::ISerializable
{
	AS_CLASSDEF(IntDouble)
	AS_MEMBERS
		AS_ITEM(int, intItem)
		AS_ITEM(double, doubleItem)
	AS_CLASSEND;
public:
	IntDouble() { intItem=0; doubleItem=0; }
	IntDouble(int i, double d) { intItem=i; doubleItem=d; }
};

class IntBuffer : public autoserial::ISerializable
{
	AS_CLASSDEF(IntBuffer)
	AS_MEMBERS
		AS_ITEM(unsigned int, bufSize)
		AS_DYNARRAY(int, buf, bufSize)
	AS_CLASSEND;
public:
	IntBuffer()
	{
		bufSize = 0;
		buf = NULL;
		//printf("new IntBuffer() buf=%p\n",buf);
	}

	IntBuffer(unsigned int size)
	{
		bufSize = size;
		buf = new int[bufSize];
		for (unsigned int i=0; i < bufSize; ++i)
			buf[i] = i;
		//printf("new IntBuffer(size) buf=%p\n",buf);
	}

	IntBuffer(const IntBuffer& other)
	{
		bufSize = other.bufSize;
		buf = new int[bufSize];
		memcpy(buf, other.buf, bufSize*sizeof(int));
		//printf("new IntBuffer(other) from buf=%p to buf=%p\n",other.buf,buf);
	}

	~IntBuffer()
	{
		//printf("deleting IntBuffer() buf=%p\n",buf);
		if (buf != NULL)
			delete[] buf;
	}
};

class Misc : public autoserial::ISerializable
{
	AS_CLASSDEF(Misc)
	AS_MEMBERS
		AS_ITEM(int, intItem1)
		AS_ITEM(int, intItem2)
		AS_ITEM(int, intItem3)
		AS_ITEM(double, doubleItem1)
		AS_ITEM(double, doubleItem2)
		AS_ITEM(double, doubleItem3)
	AS_CLASSEND;
};

/*class MiscBuffer : public autoserial::ISerializable
{
	AS_CLASSDEF(MiscBuffer)
	AS_MEMBERS
		AS_ITEM(unsigned int, bufSize)
		AS_DYNARRAY(Misc, buf, bufSize)
	AS_CLASSEND;
public:
	MiscBuffer() { bufSize = 0; buf = NULL; }

	MiscBuffer(unsigned int size)
	{
		bufSize = size;
		buf = new Misc[bufSize];
		for (unsigned int i=0; i < bufSize; ++i)
			buf[i] = Misc();
	}

	~MiscBuffer() { delete[] buf; }
};*/

class StdVector : public autoserial::ISerializable
{
	AS_CLASSDEF(StdVector)
	AS_MEMBERS
		AS_ITEM(std::vector<int>, vector)
	AS_CLASSEND;
public:
	StdVector() { }

	StdVector(unsigned int size)
	{
		for (unsigned int i=0; i < size; ++i)
			vector.push_back(i);
	}
};

class MapOfIntBuffers : public autoserial::ISerializable
{
	typedef std::map<int, IntBuffer> intbuffermap;
	AS_CLASSDEF(MapOfIntBuffers)
	AS_MEMBERS
		AS_ITEM(intbuffermap, map)
	AS_CLASSEND;
public:
	MapOfIntBuffers() { }

	MapOfIntBuffers(unsigned int size)
	{
		for (unsigned int i=0; i < size; ++i)
			map.insert(std::pair<int,IntBuffer>(i,IntBuffer(i)));
	}
};

class LinkedList : public autoserial::ISerializable
{
	AS_CLASSDEF(LinkedList)
	AS_MEMBERS
		AS_ITEM(int, element)
		AS_POINTER(LinkedList, next)
	AS_CLASSEND;
public:
	LinkedList() { element = 0; next = NULL; }

	LinkedList(int size)
	{
		element = size;
		if (size == 0)
			next = NULL;
		else
		{
			LinkedList *head = this;
			for (int i = size-1; i > 0; --i)
			{
				head->next = new LinkedList();
				head->next->element = i;
				head = head->next;
			}
		}
	}

	~LinkedList()
	{
		if (next == NULL)
			return;
		LinkedList *current = next;
		while (current->next != NULL)
		{
			LinkedList *tmp = current;
			current = current->next;
			tmp->next = NULL;
			delete tmp;
		}
	}
};

#endif
