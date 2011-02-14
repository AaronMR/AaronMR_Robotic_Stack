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

#include <autoserial/binaryhandlers.h>

#include <iostream>
#include <iomanip>

/* This object contains an int and a double */
class SimpleObject : public autoserial::ISerializable
{
	AS_CLASSDEF(SimpleObject)
	AS_MEMBERS
		AS_ITEM(int, intItem)
		AS_ITEM(double, doubleItem)
	AS_CLASSEND;

public :
	SimpleObject(int a = 0, double b = .0)
	{
		intItem = a;
		doubleItem = b;
	}

	void print()
	{
		std::cout << "  * * * * SimpleObject * * * *" << std::endl;
		std::cout << "    intItem = " << intItem << std::endl;
		std::cout << "    doubleItem = " << doubleItem << std::endl;
	}
};

/* This object contains an int and a vector containing vectors of doubles */
class ListObject : public autoserial::ISerializable
{
	AS_CLASSDEF(ListObject)
	AS_MEMBERS
		AS_PRIVATEITEM(int, nbOfDoubles)
		AS_PRIVATEITEM(std::vector<std::vector<double> >, vectorOfVectors)
	AS_CLASSEND;

public :
	ListObject() { }

	/*! The constructor without parameters is called upon deserialization
		so we keep time consuming initialisations elsewhere 
	*/
	void init()
	{
		int maxrow = 10;
		int maxcol = 5;
		srand(14);

		nbOfDoubles = 0;

		// The number of vectors and the number of doubles within
		// each vector is generated randomly
		double valr = (double)rand()/RAND_MAX;
		maxrow = (int)(valr*maxrow+5);

		for (int r = 0; r < maxrow; ++r)
		{
			valr = (double)rand()/RAND_MAX;
			maxcol = (int)(valr*maxcol+5);

			// Fill a new vector
			std::vector<double> v;
			for (int c = 0; c < maxcol; ++c)
				v.push_back((double)rand()/RAND_MAX);
			vectorOfVectors.push_back(v);
			nbOfDoubles += (int)v.size();
		}
	}

	void print()
	{
		std::cout << "  * * * * ListObject * * * *" << std::endl;
		std::cout << "    nbOfDoubles = " << nbOfDoubles << std::endl;
		for (std::vector<std::vector<double> >::iterator itv = vectorOfVectors.begin(); itv != vectorOfVectors.end(); ++itv)
        {
			std::cout << "   ";
			std::vector<double> v=*itv;
			for (std::vector<double>::iterator it = v.begin(); it != v.end(); ++it)
			{
				// Print doubles with two decimals
				std::cout << setiosflags(std::ios::fixed) << std::setprecision(2) << " " << *it;
			}
			std::cout << std::endl;
        }
		std::cout << "  * * * * * * * * * * * * * *" << std::endl;
	}

	int countDoubles()
	{
		int count = 0;
		for (size_t i = 0; i < vectorOfVectors.size(); ++i)
			count += (int)vectorOfVectors[i].size();
		return count;
	}
};
