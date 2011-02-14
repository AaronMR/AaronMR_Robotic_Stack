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

#include <string.h>
#pragma intrinsic (memcpy, memset, memcmp)

#include "timer.h"
#include "objects.h"
#include <iostream>
#include <sstream>

std::string formatSize(Size size)
{
	std::stringstream s;
	if (size < 1000)
		s << size << " bytes";
	else if (size < 1000000)
		s << size/1000.0 << " kB";
	else if (size < 1000000000)
		s << size/1000000.0 << " MB";
	else
		s << size/1000000000.0 << " GB";
	return s.str();
}

std::string formatRate(double time, Size size, int runs)
{
	double objectrate = runs/time;
	std::stringstream s;
	s << objectrate << " objects/s";
	double byterate = (size*runs)/time;
	if (byterate < 1000)
		s << " (" << byterate << " bytes/s)";
	else if (byterate < 1000000)
		s << " (" << byterate/1000.0 << " kB/s)";
	else if (byterate < 1000000000)
		s << " (" << byterate/1000000.0 << " MB/s)";
	else
		s << " (" << byterate/1000000000.0 << " GB/s)";
	return s.str();
}

int *flushBuffer = NULL;
int flushBufferSize = 2500000; // 10MB buffer
double flushTime = 0;
int dummy;
#define FLUSH() memset(flushBuffer, 0, flushBufferSize);

void benchmark(autoserial::ISerializable *object, int runs)
{
	autoserial::OpaqueObject o;
	o.set(object);
	Size size = o.getSize();
	std::cout << "Benching " << object->getTypeNameV() << " of " << formatSize(size) << std::endl;

	{
		// With alloaction + deallocation (closer to what happens in reality)
		double st = Timer::get();
		for (int i=0; i < runs; ++i)
		{
			autoserial::OpaqueObject obj;
			obj.set(object);
		}
		double t = Timer::get()-st;
		std::cout << "Serialization in " << t/runs << " ms" << std::endl;
		std::cout << "  Serialization including (de)allocation: " << formatRate(t, size, runs) << std::endl;
	}
	{
		// With alloaction + deallocation (closer to what happens in reality)
		double st = Timer::get();
		for (int i=0; i < runs; ++i)
		{
			FLUSH();
			autoserial::OpaqueObject obj;
			obj.set(object);
		}
		double t = Timer::get()-st - flushTime*runs;
		std::cout << "Serialization in " << t/runs << " ms" << std::endl;
		std::cout << "  Serialization including (de)allocation: " << formatRate(t, size, runs) << std::endl;
	}
		{
		// Deserialization+deallocation
		double st = Timer::get();
		for (int i=0; i < runs; ++i)
		{
			autoserial::ISerializable *obj = o.get();
			delete obj;
		}
		double t = Timer::get()-st;
		std::cout << "DESerialization in " << t/runs << " ms" << std::endl;
		std::cout << "  Deserialization including (de)allocation: " << formatRate(t, size, runs) << std::endl;
	}
	{
		// Deserialization+deallocation
		double st = Timer::get();
		for (int i=0; i < runs; ++i)
		{
			FLUSH();
			autoserial::ISerializable *obj = o.get();
			delete obj;
		}
		double t = Timer::get()-st - flushTime*runs;
		std::cout << "DESerialization in " << t/runs << " ms" << std::endl;
		std::cout << "  Deserialization including (de)allocation: " << formatRate(t, size, runs) << std::endl;
	}

	{
		// Comparison
		autoserial::ISerializable *obj = o.get();
		int count = 0;
		double st = Timer::get();
		for (int i=0; i < runs; ++i)
		{
			FLUSH();
			if (!obj->equals(object))
				count++;
		}
		double t = Timer::get()-st - flushTime*runs;
		std::cout << "  " << count << " Comparison time: " << formatRate(t, size, runs) << std::endl;
	}
}

int main(int argc, char *argv[])
{
	std::cout << "========= Benchmarking autoserial =========" << std::endl;
	std::cout << " All times are in seconds " << std::endl;

	// Compute rough memory bandwidth
	{
#define bufsize 100000000
		int iter = 100;
		char *buf1 = new char[bufsize];
		char *buf2 = new char[bufsize];
		// Touch the memory
		memcpy(buf1, buf2, bufsize*sizeof(char));
		double st = Timer::get();
		for (int i = 0; i < iter; ++i)
			memcpy(buf1, buf2, bufsize*sizeof(char));
		double t = Timer::get() - st;
		std::cout << "Memory bandwidth (large memcpy): " << formatRate(t, bufsize, iter) << std::endl;
		st = Timer::get();
		for (int i = 0; i < iter; ++i)
			memset(buf1, 0, bufsize*sizeof(char));
		t = Timer::get() - st;
		std::cout << "Memory bandwidth (large memset): " << formatRate(t, bufsize, iter) << std::endl;
		st = Timer::get();
		int count = 0;
		for (int i = 0; i < iter; ++i)
			count += memcmp(buf1, buf2, bufsize*sizeof(char));
		t = Timer::get() - st;
		std::cout << "Memory bandwidth (large memcmp): " << formatRate(t, bufsize, iter) << "   " << count << std::endl;
	}

	// Compute rough memory bandwidth
	{
#undef bufsize
#define bufsize 100
		int iter = 100000000;
		char *buf1 = new char[bufsize];
		char *buf2 = new char[bufsize];
		// Touch the memory
		memcpy(buf1, buf2, bufsize*sizeof(char));
		double st = Timer::get();
		for (int i = 0; i < iter; ++i)
			memcpy(buf1, buf2, bufsize*sizeof(char));
		double t = Timer::get() - st;
		std::cout << "Memory bandwidth (large memcpy): " << formatRate(t, bufsize, iter) << std::endl;
		st = Timer::get();
		for (int i = 0; i < iter; ++i)
			memset(buf1, 0, bufsize*sizeof(char));
		t = Timer::get() - st;
		std::cout << "Memory bandwidth (large memset): " << formatRate(t, bufsize, iter) << std::endl;
		st = Timer::get();
		int count = 0;
		for (int i = 0; i < iter; ++i)
			count += memcmp(buf1, buf2, bufsize*sizeof(char));
		t = Timer::get() - st;
		std::cout << "Memory bandwidth (large memcmp): " << formatRate(t, bufsize, iter) << "   " << count << std::endl;
	}

	// Initialize flushBuffer, to be read after every serialization to flush the cache
	flushBuffer = (int*)malloc(flushBufferSize);
	memset(flushBuffer, 0, flushBufferSize);
	// Get reference time, to be subtracted to each measurement
	flushTime = Timer::get();
	for (int i=0; i<100; ++i)
		FLUSH();
	flushTime = (Timer::get()-flushTime)/100;

	// Define various types of serializable objects
	{
		IntDouble d(0, 1.3);
		benchmark(&d, 1000);
	}

	{
		int a[] = {10, 100, 1000, 10000};
		for (size_t i = 0; i<sizeof(a)/sizeof(int); ++i)
		{
			IntBuffer b(a[i]);
			benchmark(&b, 1000);
		}
	}

	{
		Misc m;
		benchmark(&m, 10000);
	}

/*	{
		MiscBuffer mb(100);
	}
*/
	{
		int a[] = {10, 100, 1000, 10000};
		for (size_t i = 0; i<sizeof(a)/sizeof(int); ++i)
		{
			autoserial::internal::Buffer<Misc> b;
			b.resize(a[i]);
			benchmark(&b, 100000/a[i]);
		}
	}

	{
		int a[] = {10, 100, 1000, 10000};
		for (size_t i = 0; i<sizeof(a)/sizeof(int); ++i)
		{
			StdVector s(a[i]);
			benchmark(&s, 1000);
		}
	}

	{
		int a[] = {10, 100, 1000, 10000};
		for (size_t i = 0; i<sizeof(a)/sizeof(int); ++i)
		{
			MapOfIntBuffers m(a[i]);
			benchmark(&m, 1000/a[i]);
		}
	}

	{
		int a[] = {10, 100, 1000, 10000};
		for (size_t i = 0; i<sizeof(a)/sizeof(int); ++i)
		{
			LinkedList l(a[i]);
			benchmark(&l, 1000000/a[i]);
		}
	}

	return 0;
}
