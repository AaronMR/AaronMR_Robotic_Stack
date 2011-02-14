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

/*!	\file classregistry.h
	\brief Class registry

	Registers classes and provides unique identifiers
*/

#ifndef INCLUDED_CLASSREGISTRY_H
#define INCLUDED_CLASSREGISTRY_H

#include "defs.h"

namespace autoserial
{
	namespace internal
	{
		//! Items stored in registrar
		/*! \internal
		*/
		class Registrar
		{
		public:
			//! Next item in list
			Registrar *next;
			//! Item name
			const char *name;
			//! Item index
			Size index;

		public:
			//! Constructor
			Registrar(const char *name) { setup(name); }

			//! Get index in Registrar
			Size getTypeIndex() const { return index; }

			//! Setup item (identical to constructor)
			void setup(const char *name);
		};

		//! Specialized item stored in registrar
		/*! \internal
		*/
		template<typename regc> class GenericRegistrar : public Registrar
		{
		public:
			//! Constructor
			GenericRegistrar() : Registrar(regc::getTypeName()) { } 

			//! Setup item (identical to constructor)
			void setup() { Registrar::setup(regc::getTypeName()); }
		};

		//! Holder for registrar item singleton
		/*! \internal
		*/
		template<typename regc> class GenericRegistrarHolder
		{
		private:
			//! Held registrar item
			static GenericRegistrar<regc> item;
		public:
			//! Constructor
			GenericRegistrarHolder() { item.setup(); }

			//! Get index in Registrar
			static Size getTypeIndex() { return item.getTypeIndex(); }
		};

		//! Registrar singletons
		/*! \internal
		*/
		template<typename regc> GenericRegistrar<regc> GenericRegistrarHolder<regc>::item;
		//! Registrar singletons


		//! Autoserial class factory
		/*! \internal
		*/
		class ClassRegistry
		{
		private:
			//! First registered element
			static Registrar *head;
			//! Last registered element
			static Registrar *tail;
			//! Number of registered elements
			static Size count;
			//! Flattened view of registrar
			static Registrar **items;
		public:
			//! Add item to registrar
			/*! The constructor is added to the class factory. Due to the nasty 
				techniques used for generating the constructor, they might be added
				multiple times, so this method has to filter out these multiple
				insertions.
				\param it Constructor to add to the class factory
			*/
			static void addItem(Registrar *it)
			{
				for(Registrar *c=head;c;c=c->next)
				{
					if(!strcmp(c->name,it->name))
					{
						if(it!=c)
							it->next=NULL;
						it->index=c->index;
						return;
					}
				}
				if(tail)
					tail->next=it;
				else
					head=it;
				it->next=NULL;
				it->index=count;
				tail=it;
				count++;
			}

			//! Generate flat view
			/*!	Transforms the chained list storing the registered elements
				into an array, which is more convenient to use afterwards.
			*/
			static void start()
			{
				items=(Registrar**)malloc(count*sizeof(Registrar*));
				Size cnt=0;
				for(Registrar *c=head;c;c=c->next)
					items[cnt++]=c;
			}

			//! Delete flat view
			static void destroy()
			{
				// registrarItems are statically allocated, so there is no
				// need to free them.
				free(items);
			}

			//! Get number of registered types
			static Size length()
			{
				return count;
			}

			//! Get index of a named type
			/*! \param cname Name of the class to find
				\return Index of the class constructor, -1 if not found
			*/
			static Size indexOf(const char *cname)
			{
				// Find the correct type in the chain of registered types
				for(Size i=0;i<count;i++)
				{
					if(!strcmp(items[i]->name,cname))
						return i;
				}
				return (Size)-1;
			}

			//! Get the name of the type at index i
			/*! \param i Index to look up
			*/
			static const char *getName(Size i)
			{
				return items[i]->name;
			}
		};

//! Register a type under its type name
/*! If for some reason the internal registration mechanism should fail (i.e. the
	class factory cannot create an object of a given type, this macro can be used 
	in the global scope in order to force registration of a class.
*/
#define USETYPE(cname)			static autoserial::internal::GenericRegistrarHolder<cname > registerclass##__LINE__

#define CLASSTYPE(x) static Size getTypeIndex() { static autoserial::internal::GenericRegistrarHolder<x > gch; return gch.getTypeIndex(); } virtual Size getTypeIndexV() const { return getTypeIndex(); }

// A class must not contain both AS_CLASSDEF and AS_IDENTIFY macros, they are
// redundant
#define AS_TYPENAMESTD(tc) public: typedef tc containerType; private: class ClassdefAndIdentifyAreRedundant { }; class EnsureCorrectIdentifyArgument##tc { }; typedef tc::EnsureCorrectIdentifyArgument##tc EnsureCorrectIdentifyArgument; public: static const char *getTypeName() { return #tc; } 
#define AS_TYPENAMETEMPLATE1(tc,p1) public: typedef tc containerType; private: class ClassdefAndIdentifyAreRedundant { }; class EnsureCorrectIdentifyArgument##tc { }; typedef typename tc::EnsureCorrectIdentifyArgument##tc EnsureCorrectIdentifyArgument; public: static const char *getTypeName() { RETSTRCAT3(#tc"<",autoserial::typehandler::TypeHandler<p1>::name(),">"); } 
#define AS_TYPENAMETEMPLATE2(tc,p1,p2) public: typedef tc containerType; private: class ClassdefAndIdentifyAreRedundant { }; class EnsureCorrectIdentifyArgument##tc { }; typedef typename tc::EnsureCorrectIdentifyArgument##tc EnsureCorrectIdentifyArgument; public: static const char *getTypeName() { RETSTRCAT5(#tc"<",autoserial::typehandler::TypeHandler<p1>::name(),",",autoserial::typehandler::TypeHandler<p2>::name(),">"); } 

#define AS_IDENTIFYSTATIC(a)  
#define AS_IDENTIFYVIRTUAL(tc) virtual const char *getTypeNameV() const { return getTypeName(); } 	virtual Size getSizeV() const { return sizeof(tc); } CLASSTYPE(tc); 

	}
}

#endif

