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

#ifndef TIMER_INCLUDED_H
#define TIMER_INCLUDED_H

/*
 * Define a helper class to measure time
 * Two versions are provided for UNIX and Windows platforms
 */

#ifndef WIN32

// UNIX code
#include <sys/time.h>
class Timer
{
public:
	/* 
	 * Returns the number of seconds since the Epoch
	 * (00:00:00 UTC, January 1, 1970)
	 */
	static double get()
	{
		// Use 'man gettimeofday' for info on timeval structure
		// and gettimeofday functions
		struct timeval t;
		gettimeofday(&t,0);
		return t.tv_sec+((double)t.tv_usec)/1000000;
	}
};

#else

// Windows code
#define _WINSOCKAPI_ // Prevents inclusion of winsock.h
#include <windows.h>
class Timer
{
public:
	/* 
	 * Returns the number of seconds since the sytem was started
	 */
	static double get()
	{
		static __int64 frequency = 0;
		if (frequency == 0)
			QueryPerformanceFrequency((LARGE_INTEGER*)&frequency);
		__int64 time;
		QueryPerformanceCounter((LARGE_INTEGER*)&time);
		return time/(double)frequency;
	}
};

#endif

#endif

