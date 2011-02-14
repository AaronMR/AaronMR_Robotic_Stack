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


#ifndef INCLUDED_COMPARATOR_H
#define INCLUDED_COMPARATOR_H

#include <vector>
#include <map>

namespace autoserial
{
	class ISerializable;
	class Member;

	class Comparator
	{
	private:
		//! Vector of references for reference object
		std::vector<const ISerializable *> refs1;
		//! Vector of references for comparable object
		std::vector<const ISerializable *> refs2;

		//! Helper structure to store object tree
		std::vector<Member *> currentMembers;
		//! Stores leaves that have not been visited yet
		std::map<int,Member *> referencedMembers;

		//! Prints an error message
		static void incorrectEqualityCheck(const char *identifier)
		{
			std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;
			std::cout << "  Error found while checking equality of objects" << std::endl;
			std::cout << "    Check that enterMember/leaverMember match in " << std::endl;
			std::cout << "    " << identifier << "::compareV()" << std::endl;
			std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;
		}

		//! Main object comparison function
		/*! \param item1 First object
			\param item2 Second object
			\param accessMap If not NULL, map filled with detailed equality of every member
			\param print Whether the result should be printed on screen
			\param maxDepth Maximum recursion level for printing
		*/
		bool compare(const ISerializable *item1, const ISerializable *item2, std::map<std::string,bool> *accessMap, bool print, int maxDepth);

		//! Epsilon for double comparison
		double epsilonDouble;

		//! Epsilon for float comparison
		float epsilonFloat;

	public:
		//! Default constructor
		Comparator() { epsilonDouble=0; epsilonFloat=0; }

		//! Set/get epsilon for double comparisons
		void setEpsilonDouble(double epsilon) { epsilonDouble=epsilon; }
		double getEpsilonDouble() const { return epsilonDouble; }

		//! Set/get epsilon for float comparisons
		void setEpsilonFloat(float epsilon) { epsilonFloat=epsilon; }
		float getEpsilonFloat() const { return epsilonFloat; }

		//! Provides notification that a new member is being visited
		/*! \param name Member identifier
		*/
		void enterMember(const char *name);

		//! Provides notification that the traversal of a member completed
		/*! \param name Member identifier
		*/
		void leaveMember(const char *name);

		//! Provides notification whether current member has been modified
		/*! \param equal true if the the corresponding members from the compared objects are equal
		*/
		void memberEqual(bool equal);

		//! Tells whether the two objects are different.
		/*! \param item1 Object 1
			\param item1 Object 2
		*/
		bool areEqual(const ISerializable *item1, const ISerializable *item2);

		//! Compares two serializable objects, and returns a map telling for each member
		//!	whether its value is different in item1 and item2.
		/*! \param item1 Object 1
			\param item1 Object 2
		*/
		std::map<std::string,bool> getEqualityMap(const ISerializable *item1, const ISerializable *item2);

		//! Prints object members and tells whether they are equal in both objects.
		/*! \param item1 Object 1
			\param item1 Object 2
			\param maxLevel How deep members should be printed (e.g., if objects contain vectors of vectors of objects)
		*/
		void printMembersEquality(const ISerializable *item1, const ISerializable *item2, int maxDepth=-1);

		//! Comparison function for pointed objects
		void areEqualRef(const ISerializable *item1, const ISerializable *item2);
	};
}

#endif
