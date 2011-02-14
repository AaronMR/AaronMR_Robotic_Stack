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

/*!	\file defs.h
	\brief Base declarations for autoserial

	Basic declarations for types, interfaces and error handling.
*/

#ifndef INCLUDED_DEFS_H
#define INCLUDED_DEFS_H

//
// Version number
//
#define AS_VERSION_MAJOR	1
#define AS_VERSION_MINOR	0
#define AS_VERSION_BUILD	1

//
// Simple compiler and platform detection
//
#if defined(_MSC_VER)

# define AS_PLATFORM_WIN32

#elif defined(WIN32)
// This is mingw32 
# define AS_PLATFORM_WIN32

#else
// This is plain UNIX
# define AS_PLATFORM_POSIX
#endif

#ifdef _MSC_VER // Cannot use COMPILER_MS here, since 13.10 has COMPILER_GCC defined
// Some of these must be defined in advance of including the standard
// header files in order to avoid those pesky warnings about excessively
// long identifiers (which this code generates in large quantities).
# pragma warning(disable:4786)	// identifier name was truncated to 'x' characters
# pragma warning(disable:4503)	// decorated name length exceeded, name was truncated
# pragma warning(disable:4355)	// 'this' used in base member initializer list
# if _MSC_VER >= 1400 // If we are using VS 2005 or greater
#  define _CRT_SECURE_NO_DEPRECATE
#  define _SCL_SECURE_NO_DEPRECATE
# endif
#endif


#ifdef AS_PLATFORM_WIN32
// Windows specific include files
# ifndef _WIN32_WINNT
// If there is no previous WINNT version defined, define at least NT 4.0
#  define _WIN32_WINNT 0x0403
# endif

# include <winsock2.h>
# include <windows.h>
# include <io.h>
# include <direct.h>
# include <malloc.h>
# include <stdio.h>
# include <stdlib.h>

// Undefine min and max (these should not be macros anyway)
# undef min
# undef max
# if _MSC_VER<1300
// For older MS compilers, set min and max to _MIN and _MAX
#  define min _MIN
#  define max _MAX
# endif
#else
// UNIX include files
//# include <string.h>
//# include <pthread.h>
//# include <sys/timeb.h>
//# include <unistd.h>
//# include <errno.h>
//# include <sys/time.h>
//# include <signal.h>
//# include <sys/types.h>
//# include <sys/socket.h>
//# include <netinet/in.h>
//# include <netinet/tcp.h>
//#include <dlfcn.h>
//# include <netdb.h>
//# include <unistd.h>
//# include <alloca.h>

#endif

//#if defined(__GLIBCPP__)
#if defined(__GNUG__)
// For the interlocked increment/decrement functions of gcc
#include <bits/atomicity.h>
#endif



// Standard includes
//#include <assert.h>

#ifdef AS_DEBUG
namespace autoserial
{
	//! Function called on assert failures
	void localAssert(const char *exp, const char *file, int line)
	{
		std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
		std::cout << "FATAL ERROR: Assertion " << exp << " failed on " << file << ":" << line;
		std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
	}
}
#define assert(exp) (void)( (exp) || (autoserial::localAssert(#exp, __FILE__, __LINE__), 0) )
#else
#define assert(exp) ((void)0)
#endif

// STL includes
#include <string>
#include <list>
#include <iostream>
#include <fstream>
#include <sstream>
//#include <map>
//#include <vector>
#include <deque>
#include <set>
//#include <queue>
//#include <iomanip>

//#if defined(AS_PLATFORM_WIN32)&&!defined(AS_SYS_TYPES)
//! 8 bit signed integer
typedef char Int8;
//! 16 bit signed integer
typedef short Int16;
//! 32 bit signed integer
typedef int Int32;
//! 8 bit unsigned integer
typedef unsigned char UInt8;
//! 16 bit unsigned integer
typedef unsigned short UInt16;
//! 32 bit unsigned integer
typedef unsigned int UInt32;
/*#else
//! 8 bit signed integer
typedef int8_t Int8;
//! 16 bit signed integer
typedef int16_t Int16;
//! 32 bit signed integer
typedef int32_t Int32;
//! 8 bit unsigned integer
typedef u_int8_t UInt8;
//! 16 bit unsigned integer
typedef u_int16_t UInt16;
//! 32 bit unsigned integer
typedef u_int32_t UInt32;
#endif
*/
//! 32 bit floating point number
typedef float Float;
//! 64 bit floating point number
typedef double Double;
//! Integer type for storing sizes of data structures
typedef size_t Size;

//! Integer type for storing sizes of data structures in network packets 
/*! Since size_t might be 32 or 64 bits (or maybe even something completely 
	different), we have to decide what to use when serializing data in
	order to be consistent across multiple platforms. For now, it seems
	reasonable to stick to 32 bits, since transferring data blocks in excess
	of 4 GB over typical current networks would take a very long time, and is
	thus unlikely in the typical Autoserial usage scenario.
*/
typedef UInt32 XPSize;

//! Boolean type
typedef bool Bool;

#ifdef _MSC_VER
#  ifdef AS_SYS_TYPES
//! 64 bit signed integer
typedef int64_t Int64;
//! 64 bit unsigned integer
typedef u_int64_t UInt64;
#  else
//! 64 bit signed integer
typedef __int64 Int64;
//! 64 bit unsigned integer
typedef unsigned __int64 UInt64;
#  endif
//! 64 bit signed integer format flag for printf
#define I64F "%I64d"
//! 64 bit unsigned integer format flag for printf
#define U64F "%I64u"
#else
//! 64 bit signed integer
typedef long long Int64;
//typedef int64_t Int64;
//! 64 bit unsigned integer
typedef unsigned long long UInt64;
//typedef u_int64_t UInt64;
//! 64 bit signed integer format flag for printf
#define I64F "%lld"
//! 64 bit unsigned integer format flag for printf
#define U64F "%llu"
#endif

#endif

