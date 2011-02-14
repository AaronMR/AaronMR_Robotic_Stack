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

/*!	\file classfactory.h
	\brief Class factory

	Registers classes and provides a class factory construction mechanism
*/

#ifndef INCLUDED_CLASSFACTORY_H
#define INCLUDED_CLASSFACTORY_H

#include "defs.h"
#include "classregistry.h"

#include "base.h"
#include "typehandler.h"

namespace autoserial
{
	namespace internal
	{

		//! Items stored in registrar
		/*! \internal
		*/
		class Constructor
		{
		public:
			//! Next item in list
			Constructor *next;
			//! Item name
			const char *name;
			//! Item index
			Size index;

		public:
			//! Constructor
			Constructor(const char *name);

			//! Destructor
			virtual ~Constructor() { }

			//! Create an object of this registered type
#ifdef AS_DEBUG_MEMORY
			virtual void *create(const char *file, int line) const = 0;
#else
			virtual void *create() const = 0;
#endif

			//! Get index in Registrar
			Size getIndex() const { return index; }
		};

		//! Specialized item stored in registrar
		/*! \internal
		*/
		template<typename regc> class GenericConstructor : public Constructor
		{
		private:
			// Check for AS_IDENTIFY macro in class declaration (compile-time)
			typedef typename regc::containerType IdentifyInType;

		public:
			//! Constructor
			GenericConstructor() : Constructor(regc::getTypeName()) { } 

			//
			// The compilation might fail here if the AS_IDENTIFY macro
			// is missing in the class declaration
			//

			//! Create an object of this registered type
#ifdef AS_DEBUG_MEMORY
#undef new
			virtual void *create(const char *file, int line) const { return new( _NORMAL_BLOCK, file, line) regc; }
#define new MYDEBUG_NEW
#else
			virtual void *create() const { return new regc; }
#endif
		};

		//! Holder for registrar item singleton
		/*! \internal
		*/
		template<typename regc> class GenericConstructorHolder
		{
		private:
			// Check for AS_IDENTIFY macro in class declaration (compile-time)
			typedef typename regc::containerType IdentifyInType;

			//! Held registrar item
			static GenericConstructor<regc> item;
		public:
			//! Constructor
			GenericConstructorHolder() { item.name=regc::getTypeName(); }

			//! Get index in Registrar
			static Size getIndex() 
			{ 
				// Assertion fails if item is not in class factory. Most probable cause
				// is two types with the same name in two different namespaces.
				assert(item.getIndex()!=(UInt32)-1); 
				return item.getIndex(); 
			}
		};

		//! Registrar singletons
		/*! \internal
		*/
		template<typename regc> GenericConstructor<regc> GenericConstructorHolder<regc>::item;

		//! Autoserial class factory
		/*! \internal
		*/
		class ClassFactory
		{
		private:
			//! First registered element
			static Constructor *head;
			//! Last registered element
			static Constructor *tail;
			//! Number of registered elements
			static Size count;
			//! Flattened view of registrar
			static Constructor **items;
			//! Indicates whether factory has been started
			static bool started;

		public:
			//! Add item to registrar
			/*! The constructor is added to the class factory. Due to the nasty 
				techniques used for generating the constructor, they might be added
				multiple times, so this method has to filter out these multiple
				insertions.
				\param it Constructor to add to the class factory
			*/
			static void addItem(Constructor *it)
			{
				it->index=(Size)-1;
				for(Constructor *c=head;c;c=c->next)
				{
					if(!strcmp(c->name,it->name))
						return;
				}
				if(started)
				{
					//DLOG((DPSLog.write(3) << "Added item " << it->name << " to class factory after startup"));
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
				// Start the registry first
				ClassRegistry::start();
				// And now create local flat list

				items=(Constructor**)malloc(count*sizeof(Constructor*));
				Size cnt=0;
				for(Constructor *c=head;c;c=c->next)
				{
					assert(c->index==cnt);
					items[cnt++]=c;
				}

				started=true;
			}

			//! Delete flat view
			static void destroy()
			{
				// registrarItems are statically allocated, so there is no
				// need to free them.
				free(items);

				// Destroy registry as well
				ClassRegistry::destroy();

				started=false;
			}

			//! Create an object of a named type
			/*! \param cname Name of the class to instantiate
			*/
#ifdef AS_DEBUG_MEMORY
			static void *create(const char *cname, const char *file, int line)
#else
			static void *create(const char *cname)
#endif
			{
				// Find the correct type in the chain of registered types
				for(Constructor *c=head;c;c=c->next)
				{
					if(!strcmp(c->name,cname))
#ifdef AS_DEBUG_MEMORY
						return c->create(file,line);
#else
						return c->create();
#endif
				}
				//DLOG((DPSLog.write(0) << "Could not create " << cname));
				return NULL;
			}

			//! Create an object of an indexed type
			/*! \param index Index of the class to instantiate in the class factory
			*/
#ifdef AS_DEBUG_MEMORY
			static void *create(Size index, const char *file, int line)
#else
			static void *create(Size index)
#endif
			{
#ifdef AS_DEBUG_MEMORY
				return items[index]->create(file,line);
#else
				return items[index]->create();
#endif
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
#define REGISTERTYPE(cname)			USETYPE(cname); static autoserial::internal::GenericConstructorHolder<cname > registeritem##__LINE__

#ifdef AS_DEBUG_MEMORY
#define CREATEARGS ,__FILE__,__LINE__
#else
#define CREATEARGS
#endif

#define AS_REGISTERCLASS(x) virtual Size getFactoryTypeIndexV() const { static autoserial::internal::GenericConstructorHolder<x > gch; return gch.getIndex(); }

#define AS_IDENTIFYNOREG(a) AS_TYPENAMESTD(a) AS_IDENTIFYVIRTUAL(a) 
#define AS_IDENTIFY(a)		 AS_TYPENAMESTD(a) AS_IDENTIFYSTATIC(a) AS_IDENTIFYVIRTUAL(a) AS_REGISTERCLASS(a) 

#define AS_TEMPLATEIDENTIFY1(a,b) AS_TEMPLATEDEF1(a,b) AS_MEMBERS AS_CLASSEND
#define AS_TEMPLATEIDENTIFY2(a,b,c) AS_TEMPLATEDEF2(a,b,c) AS_MEMBERS AS_CLASSEND
//#define AS_TEMPLATEIDENTIFY1(a,b) AS_TYPENAMETEMPLATE1(a,b) AS_IDENTIFYSTATIC(a) AS_IDENTIFYVIRTUAL(a) AS_REGISTERCLASS(a)
//#define AS_TEMPLATEIDENTIFY2(a,b,c) AS_TYPENAMETEMPLATE2(a,b,c) AS_IDENTIFYSTATIC(a) AS_IDENTIFYVIRTUAL(a) AS_REGISTERCLASS(a)


	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	//! Macro for transforming a string into a class
/*! \internal
	This silly macro is required since it is not acceptable to pass a constant string
	pointer as argument to a template. So we just wrap the string pointer in a class.
*/
#define STRING(x) struct __string_##x { static inline const char* name() { return #x; } }

//! Macro for transforming a string into a class
/*! \internal
	This silly macro is required since it is not acceptable to pass a constant string
	pointer as argument to a template. So we just wrap the string pointer in a class.
	Identical to STRING, but the created class has a different name.
*/
#define STRINGB(x) struct __stringb_##x { static inline const char* name() { return #x; } }

		//! Class for holding items in a linked list of types
		/*! \internal
			Template parameters:
			- tc: Owner class
			- cName: Member name
			- serialName: Member serialization name
			- getter: Member serialization helper class
			- next: next member in stack
		*/
		template<typename tc, typename cName, typename serialName, typename getter, typename next> struct MemberStack
		{
			//! Next item
			typedef next nextItem;
			//! Current item type
			typedef typename getter::type type;

			//! Write item to output stream
			/*template<typename T> static inline bool contains() 
			{
				if(T::getTypeIndex()==tc::getTypeIndex())
					return true;
				if(tc::contains<T>())
					return true;
				return next::contains<T>();
			}*/

			//! Write item to output stream
			static inline Result write(IOutputStream *s, const tc& base) 
			{
				Result r;
				if(AS_FAILED(r=next::write(s,base)))
					return r;
				ENTERITEM(serialName::name());
				TYPEITEM(typehandler::TypeHandler<typename getter::type>::name());
				if(AS_FAILED(r=typehandler::write(s,getter::getConst(base))))
					return r;
				LEAVEITEM(serialName::name());
				return AS_OK;
			}

			//! Read item from input stream
			static inline Result read(IInputStream *s, tc& base) 
			{ 
				Result r;
				if(AS_FAILED(r=next::read(s,base)))
					return r;
				ENTERITEM(serialName::name());
				if(AS_FAILED(r=typehandler::read(s,getter::get(base))))
					return r;
				LEAVEITEM(serialName::name());
				return AS_OK;
			}

			//! Static indexing of stack elements
			enum {index=next::index+1};

			//! Compare two items for identity
			static inline void equals(Comparator *v, const tc& item1, const tc& item2)
			{
				//Obtain type name:    typehandler::TypeHandler<typename getter::type>::name()
				//Obtain member name:  cName::name()
				next::equals(v,item1,item2);

				v->enterMember(cName::name());
				typehandler::equals(v,getter::getConst(item1),getter::getConst(item2));
				v->leaveMember(cName::name());
			}

			typedef tc ownerType;
			static const char *getBaseClassName() { return ownerType::getTypeName(); }
		};

		//! Class for holding pointer to ISerializable items in a linked list of types
		/*! \internal
			Template parameters:
			- tc: Owner class
			- cName: Member name
			- serialName: Member serialization name
			- getter: Member serialization helper class
			- next: next member in stack
		*/
		template<typename tc, typename cName, typename serialName, typename getter, typename next> struct MemberStack<tc*, cName, serialName, getter, next>
		{
			//! Next item
			typedef next nextItem;
			//! Current item type
			typedef typename getter::type type;

			//! Write item to output stream
			/*template<typename T> static inline bool contains() 
			{
				if(T::getTypeIndex()==tc::getTypeIndex())
					return true;
				if(tc::contains<T>())
					return true;
				return next::contains<T>();
			}*/

			//! Write item to output stream
			static inline Result write(IOutputStream *s, const tc& base) 
			{
				Result r;
				if(AS_FAILED(r=next::write(s,base)))
					return r;
				ENTERITEM(serialName::name());
				TYPEITEM(typehandler::TypeHandler<typename getter::type>::name());
				if(AS_FAILED(r=s->writeRef(getter::getConst(base))))//getter::getConst(base))))
					return r;
				LEAVEITEM(serialName::name());
				return AS_OK;
			}

			//! Read item from input stream
			static inline Result read(IInputStream *s, tc& base) 
			{ 
				Result r;
				if(AS_FAILED(r=next::read(s,base)))
					return r;
				ENTERITEM(serialName::name());
				if(AS_FAILED(r=s->readRef((ISerializable**)&getter::get(base))))//typehandler::read(s,getter::get(base))))
					return r;
				LEAVEITEM(serialName::name());
				return AS_OK;
			}

			//! Static indexing of stack elements
			enum {index=next::index+1};

			//! Compare two items for identity
			static inline void equals(Comparator *v, const tc& item1, const tc& item2)
			{
				//Obtain type name:    typehandler::TypeHandler<typename getter::type>::name()
				//Obtain member name:  cName::name()
				next::equals(v,item1,item2);

				v->enterMember(cName::name());
				v->areEqualRef(getter::getConst(item1),getter::getConst(item2));
				//typehandler::equals(v,getter::getConst(item1),getter::getConst(item2));
				v->leaveMember(cName::name());
			}

			typedef tc ownerType; // Shouldn't this be tc* (does not compile on Cygwin)?
			static const char *getBaseClassName() { return ownerType::getTypeName(); }
		};

		//! Class for holding pointer to dynamic arrays in a linked list of types
		/*! \internal
			Template parameters:
			- tc: Owner class
			- cName: Member name
			- serialName: Member serialization name
			- getter: Member serialization helper class
			- lengthGetter: Member storing the length of the array
			- next: next member in stack
			*/
		template<typename tc, typename cName, typename serialName, typename getter, typename lengthGetter, typename next> struct MemberArrayStack
		{
			//! Next item
			typedef next nextItem;
			//! Current item type
			typedef typename getter::type type;

			//! Write item to output stream
			static inline Result write(IOutputStream *s, const tc& base) 
			{
				Result r;
				if(AS_FAILED(r=next::write(s,base)))
					return r;
				//std::cout << "Writing type " << typehandler::TypeHandler<typename getter::type>::name() << std::endl;
				//std::cout << " array length is " << lengthGetter::getConst(base) << std::endl;
				ENTERITEM(serialName::name());
				TYPEITEM(typehandler::TypeHandler<typename getter::type>::name());
				if(AS_FAILED(r=typehandler::writeArray(s,getter::getConst(base),lengthGetter::getConst(base))))
					return r;
				LEAVEITEM(serialName::name());
				return AS_OK;
			}
			//! Read item from input stream
			static inline Result read(IInputStream *s, tc& base) 
			{ 
				Result r;
				if(AS_FAILED(r=next::read(s,base)))
					return r;
				ENTERITEM(serialName::name());
				if(AS_FAILED(r=typehandler::readArray(s,getter::get(base),lengthGetter::getConst(base))))
					return r;
				LEAVEITEM(serialName::name());
				return AS_OK;
			}

			//! Static indexing of stack elements
			enum {index=next::index+1};

			//! Compare two items for identity
			static inline void equals(Comparator *v, const tc& item1, const tc& item2)
			{
				//Obtain type name:    typehandler::TypeHandler<typename getter::type>::name()
				//Obtain member name:  cName::name()
				next::equals(v,item1,item2);

				v->enterMember(cName::name());
				typehandler::equalsArray(v,getter::getConst(item1),lengthGetter::getConst(item1),getter::getConst(item2),lengthGetter::getConst(item2));
				v->leaveMember(cName::name());
			}

			typedef tc ownerType;
			static const char *getBaseClassName() { return ownerType::getTypeName(); }
		};

		//! \brief Dummy class for last item in list
		/*!	\internal
			This type is used to terminate a MemberStack type list.
			\param tc Owner class
		*/
		template<typename tc> struct MemberStackTop
		{
			template<typename T> static inline bool contains() { return false; }
#if 1
			static inline Result write(IOutputStream *, const tc& ) { return AS_OK; }
			static inline Result read(IInputStream *, tc& ) { return AS_OK; }
#else
			static inline Bool write(IOutputStream *, const tc& ) { return true; }
			static inline Bool read(IInputStream *, tc& ) { return true;}
#endif

			enum {index=-1};
			static inline void equals(Comparator *v,const tc& item1, const tc& item2) { }
			static const char *getBaseClassName() { return NULL; }
		};

#define MEMBERGETTER(t,n) struct getter##n { typedef t type; static inline t& get(containerType& item) { return item.n; } static inline const t& getConst(const containerType& item) { return item.n; }  } 
#define AS_BASEGETTER(t,n) struct getter##n { typedef t type; static inline t& get(containerType& item) { return item; } static inline const t& getConst(const containerType& item) { return item; }  }
//#define SERIAL(t,n) struct __serialize##n { static inline void iwrite(autoserial::BinarySerializationTarget *trg, const containerType& a) { trg->write(a.n); } static inline void iread(autoserial::BinarySerializationSource *src, containerType& a) { src->read(a.n); } };
#define AS_COMMON(t,n,ren) STRING(n); STRINGB(ren); MEMBERGETTER(t,n); typedef autoserial::internal::MemberStack<containerType, __string_##n, __stringb_##ren, getter##n, __curt##n >


	// Redefinitions that also implement equals() and equalsV()
#define AS_SERIALSTATIC(tc) AS_IDENTIFYSTATIC(tc) \
	autoserial::Result write(autoserial::IOutputStream* s) const { autoserial::Result r; if(AS_FAILED(r=__baseClasses::write(s,*this))) return r; return __members::write(s,*this); } \
	autoserial::Result read(autoserial::IInputStream* s) { autoserial::Result r; if(AS_FAILED(r=__baseClasses::read(s,*this))) return r; return __members::read(s,*this);} \
	void compare(autoserial::Comparator *v, const tc& item) const { __baseClasses::equals(v,*this,item); __members::equals(v,*this,item);}

#define AS_SERIALVIRTUAL(tc) AS_IDENTIFYVIRTUAL(tc) \
	virtual autoserial::Result writeV(autoserial::IOutputStream* s) const { return this->write(s); } \
	virtual autoserial::Result readV(autoserial::IInputStream* s) { return this->read(s); } \
	virtual void compareV(autoserial::Comparator *v, const autoserial::ISerializable *s) const\
	{ if(strcmp(getTypeNameV(),s->getTypeNameV())) { v->memberEqual(false); return; } this->compare(v,*(tc*)s); } \
	virtual const char *getBaseClassNameV() const { return tc::baseClassAccessPoint::getBaseClassName(); }


#define NOBASECLASSES private: typedef autoserial::internal::MemberStackTop<containerType> __baseClasses; public: typedef __baseClasses baseClassAccessPoint 
#define AS_SIMPLECLASSDEF(tc) AS_TYPENAMESTD(tc) AS_SERIALSTATIC(tc) \
	NOBASECLASSES; typedef autoserial::MemberStackTop<containerType> 


#define AS_CUSTOMSERIAL(a) AS_TYPENAMESTD(a) AS_SERIALVIRTUAL(a) AS_IDENTIFYSTATIC(a) AS_REGISTERCLASS(a) /*typedef baseClassAccessPoint __baseClasses*/

#define AS_CLASSDEFNOREG(tc) AS_TYPENAMESTD(tc) AS_SERIALSTATIC(tc) AS_SERIALVIRTUAL(tc) \
	private: typedef autoserial::internal::MemberStackTop<containerType> 

#define AS_CLASSDEF(tc) AS_TYPENAMESTD(tc) AS_SERIALSTATIC(tc) AS_SERIALVIRTUAL(tc) AS_REGISTERCLASS(tc) \
	private: typedef autoserial::internal::MemberStackTop<containerType> 

#define AS_TEMPLATEDEF1(tc,t1) AS_TYPENAMETEMPLATE1(tc,t1) AS_SERIALSTATIC(tc) AS_SERIALVIRTUAL(tc) AS_REGISTERCLASS(tc) \
	private: typedef autoserial::internal::MemberStackTop<containerType> 
#define AS_TEMPLATEDEF2(tc,t1,t2) AS_TYPENAMETEMPLATE2(tc,t1,t2) AS_SERIALSTATIC(tc) AS_SERIALVIRTUAL(tc) AS_REGISTERCLASS(tc) \
	private: typedef autoserial::internal::MemberStackTop<containerType> 

#define AS_TEMPLATEDEFNOREG1(tc,t1) AS_TYPENAMETEMPLATE1(tc,t1) AS_SERIALSTATIC(tc) AS_SERIALVIRTUAL(tc) \
	private: typedef autoserial::internal::MemberStackTop<containerType> 
#define AS_TEMPLATEDEFNOREG2(tc,t1,t2) AS_TYPENAMETEMPLATE2(tc,t1,t2) AS_SERIALSTATIC(tc) AS_SERIALVIRTUAL(tc) \
	private: typedef autoserial::internal::MemberStackTop<containerType> 

#define AS_SINGLEBASECLASS(cn) AS_BASEGETTER(cn,id); private: typedef autoserial::MemberStack<containerType,autoserial::typehandler::TypeHandler<cn >, autoserial::typehandler::TypeHandler<cn >, getter##id, autoserial::MemberStackTop<containerType> > __baseClasses; public: typedef __baseClasses baseClassAccessPoint; 

	// AS_BASECLASS uses the __LINE__ macro. You should never put two AS_BASECLASS
	// macros on the same line, since this will generate two types with the 
	// same name, and thus a compiler error.
#define AS_BASECLASS2(cn,id) __curb##id; AS_BASEGETTER(cn,id); \
	private: typedef autoserial::MemberStack<containerType,autoserial::typehandler::TypeHandler<cn >, autoserial::typehandler::TypeHandler<cn >, getter##id, __curb##id >
#define AS_BASECLASS1(cn,id) AS_BASECLASS2(cn,id)
#define AS_BASECLASS(cn) AS_BASECLASS1(cn,__LINE__)

#define AS_MEMBERS  __baseClasses; \
	public: typedef __baseClasses baseClassAccessPoint; \
	private: typedef autoserial::internal::MemberStackTop<containerType> 
#define AS_PUBLICITEM(t,n) __curt##n; public: t n; private: AS_COMMON(t,n,n)
#define AS_PUBLICITEMR(t,n,ren) __curt##n; public: t n; private: AS_COMMON(t,n,ren)

#define AS_PRIVATEITEM(t,n) __curt##n; private: t n; private: AS_COMMON(t,n,n)
#define AS_PRIVATEITEMR(t,n,ren) __curt##n; private: t n; private: AS_COMMON(t,n,ren)
#define AS_PROTECTEDITEM(t,n) __curt##n; protected: t n; private: AS_COMMON(t,n,n)
#define AS_PROTECTEDITEMR(t,n,ren) __curt##n; protected: t n; private: AS_COMMON(t,n,ren)

#define AS_ITEM(t,n) AS_PUBLICITEM(t,n)
#define AS_ITEMR(t,n,ren) AS_PUBLICITEMR(t,n,ren)
#define AS_ITEMD(t,n) __curt##n; private: AS_COMMON(t,n,n)

#define AS_ARRAY(t,n,s) __curt##n; typedef t __arr##n[s]; public: __arr##n n; AS_COMMON(__arr##n,n,n)

// Define getter for pointed types
#define AS_POINTER_MEMBERGETTER(t,n) struct getter##n { typedef t type; static inline t*& get(containerType& item) { return item.n; } static inline t* const & getConst(const containerType& item) { return item.n; }  } 

// Members for arrays
#define AS_ARRAY_COMMON(t,n,ren,l) STRING(n); STRINGB(ren); AS_POINTER_MEMBERGETTER(t,n); typedef autoserial::internal::MemberArrayStack<containerType, __string_##n, __stringb_##ren, getter##n, getter##l, __curt##n >
#define AS_PUBLICDYNARRAY(t,n,l) __curt##n; public: t *n; private: AS_ARRAY_COMMON(t,n,n,l)
#define AS_PRIVATEDYNARRAY(t,n,l) __curt##n; private: t *n; private: AS_ARRAY_COMMON(t,n,n,l)
#define AS_PROTECTEDDYNARRAY(t,n,l) __curt##n; protected: t *n; private: AS_ARRAY_COMMON(t,n,n,l)

#define AS_DYNARRAY(t,n,l) AS_PUBLICDYNARRAY(t,n,l)

// Members for pointers to ISerializable
#define AS_POINTER_COMMON(t,n,ren) STRING(n); STRINGB(ren); AS_POINTER_MEMBERGETTER(t,n); typedef autoserial::internal::MemberStack<containerType*, __string_##n, __stringb_##ren, getter##n, __curt##n >
#define AS_PUBLICPOINTER(t,n) __curt##n; public: t *n; private: AS_POINTER_COMMON(t,n,n)
#define AS_PRIVATEPOINTER(t,n) __curt##n; private: t *n; private: AS_POINTER_COMMON(t,n,n)
#define AS_PROTECTEDPOINTER(t,n) __curt##n; protected: t *n; private: AS_POINTER_COMMON(t,n,n)

#define AS_POINTER(t,n) AS_PUBLICPOINTER(t,n)

#define AS_CLASSEND __members; public: enum { checkForBaseClasses = sizeof(__baseClasses) } 

	}
}

#endif
