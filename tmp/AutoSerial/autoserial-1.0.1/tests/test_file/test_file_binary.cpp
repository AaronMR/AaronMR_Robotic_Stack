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

//#define AS_SYS_TYPES

#include <autoserial/autoserial.h>
#include "../objects.h"

using namespace autoserial;


int main(int argc, char *argv[])
{
	const char *filename = "serialized.bin";

	/* ============================================================= */
	/*                            WRITING                            */
	/* ============================================================= */

	std::cout << "-------------------------------------" << std::endl;
	std::cout << " Binary file writer test" << std::endl;
	std::cout << "-------------------------------------" << std::endl;

	std::cout << "Opening " << filename << " for writing" << std::endl;
	BinaryFileWriter bfw(filename);
	//FlippingBinaryFileWriter bfw(filename);

	// Create and initialize the two objects
	SimpleObject so1(10, 20.0);
	ListObject lo1;
	lo1.init();

	std::cout << std::endl << "Writing SimpleObject:" << std::endl;
	so1.print();

	if (AS_FAILED(bfw.write(&so1)))
	{
		std::cout << "Could not write SimpleObject in file " << filename << std::endl;
		return 1;
	}

	std::cout << std::endl << "Writing ListObject:" << std::endl;
	lo1.print();
	if (AS_FAILED(bfw.write(&lo1)))
	{
		std::cout << "Could not write ListObject in file " << filename << std::endl;
		return 1;
	}

	std::cout << "Serialization successful" << std::endl << std::endl;



	/* ============================================================= */
	/*                            READING                            */
	/* ============================================================= */

	std::cout << "-------------------------------------" << std::endl;
	std::cout << " Binary file reader test" << std::endl;
	std::cout << "-------------------------------------" << std::endl;

	std::cout << "Opening " << filename << " for reading" << std::endl;
	BinaryFileReader bfr(filename);
	//FlippingBinaryFileReader bfr(filename);

	SimpleObject *so2;
	ListObject *lo2;

	std::cout << std::endl << "Reading SimpleObject:" << std::endl;
	if (AS_FAILED(bfr.read((ISerializable**)&so2)))
	{
		std::cout << "Could not read SimpleObject from file " << filename << std::endl;
		return 1;
	};
	so2->print();

	std::cout << std::endl << "Reading ListObject:" << std::endl;
	if (AS_FAILED(bfr.read((ISerializable**)&lo2)))
	{
		std::cout << "Could not read SimpleObject from file " << filename << std::endl;
		return 1;
	};
	lo2->print();

	//if (lo2->isSubclassOf<ISerializable>())
	//	std::cout << "all your base are belong to use" << std::endl;
	std::cout << "Deserialization successful" << std::endl;

	// Checking that written and read objects are equal
	if (so1.equals(so2))
		std::cout << "Simple objects are identical" << std::endl;
	else
	{
		Comparator c;
		c.printMembersEquality(&so1, so2);
	}
	if (lo1.equals(lo2))
		std::cout << "List objects are identical" << std::endl;
	else
	{
		Comparator c;
		c.printMembersEquality(&lo1, lo2);
	}

	return 0;
}
