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

/*!	\file typehandler.h
	\brief Type handler classes

	These classes provide type handlers for all basic types, and for STL containers.
*/

#ifndef INCLUDED_TYPEHANDLER_H
#define INCLUDED_TYPEHANDLER_H

#include "comparator.h"

#ifdef WIN32
// Disable safety warnings
#pragma warning(disable : 4996)
#endif

namespace autoserial
{
#ifdef AS_NAMED_SERIALIZATION
#define ENTERITEM(x) s->enter(x)
#define LEAVEITEM(x) s->leave(x)
#define TYPEITEM(x) s->type(x)
#else
#define ENTERITEM(x)
#define LEAVEITEM(x)
#define TYPEITEM(x)
#endif

	//! Classes for handling types for reflection and serialization
	/*! \internal This Namespace contains all classes used to handle types that do not have
		built-in reflection capabilities. This includes all the basic types (int, float,
		etc.) and also the STL containers and utility classes.
	*/
	namespace typehandler
	{
		//! String constant class
		/*! \internal This class is used in typehandler implementations that need to
			dynamically construct the names of their corresponding types
			(for example templates). It ensures proper destruction of the
			strings at program exit.
		*/
		class ConstString
		{
		public:
			//! String
			char *str;
		public:
			//! Constructor
			ConstString() { str=NULL; }
			//! Destructor
			~ConstString() { if(str) free(str); }
		};
//! Static concatenation of 2 strings
/*! \internal Returns a static string that is a concatenation of the arguments
	\param s1 1st string
	\param s2 2nd string
	\return Concatenation of both strings
*/
#define RETSTRCAT2(s1,s2) static autoserial::typehandler::ConstString tn; if(tn.str==NULL) { \
	tn.str=(char*)malloc(strlen(s1)+strlen(s2)+1); \
	strcpy(tn.str,s1); strcat(tn.str,s2); \
	} return tn.str
//! Static concatenation of 3 strings
/*! \internal Returns a static string that is a concatenation of the arguments
	\param s1 1st string
	\param s2 2nd string
	\param s3 3rd string
	\return Concatenation of the three strings
*/
#define RETSTRCAT3(s1,s2,s3) static autoserial::typehandler::ConstString tn; if(tn.str==NULL) { \
	tn.str=(char*)malloc(strlen(s1)+strlen(s2)+strlen(s3)+1); \
	strcpy(tn.str,s1); strcat(tn.str,s2); strcat(tn.str,s3); \
	} return tn.str
//! Static concatenation of 4 strings
/*! \internal Returns a static string that is a concatenation of the arguments
	\param s1 1st string
	\param s2 2nd string
	\param s3 3rd string
	\param s4 4th string
	\return Concatenation of the four strings
*/
#define RETSTRCAT4(s1,s2,s3,s4) static autoserial::typehandler::ConstString tn; if(tn.str==NULL) { \
	tn.str=(char*)malloc(strlen(s1)+strlen(s2)+strlen(s3)+strlen(s4)+1); \
	strcpy(tn.str,s1); strcat(tn.str,s2); strcat(tn.str,s3); strcat(tn.str,s4); \
	} return tn.str
//! Static concatenation of 5 strings
/*! \internal Returns a static string that is a concatenation of the arguments
	\param s1 1st string
	\param s2 2nd string
	\param s3 3rd string
	\param s4 4th string
	\param s5 5th string
	\return Concatenation of the five strings
*/
#define RETSTRCAT5(s1,s2,s3,s4,s5) static autoserial::typehandler::ConstString tn; if(tn.str==NULL) { \
	tn.str=(char*)malloc(strlen(s1)+strlen(s2)+strlen(s3)+strlen(s4)+strlen(s5)+1); \
	strcpy(tn.str,s1); strcat(tn.str,s2); strcat(tn.str,s3); strcat(tn.str,s4); strcat(tn.str,s5); \
	} return tn.str
//! Static concatenation of 7 strings
/*! \internal Returns a static string that is a concatenation of the arguments
	\param s1 1st string
	\param s2 2nd string
	\param s3 3rd string
	\param s4 4th string
	\param s5 5th string
	\param s6 6th string
	\param s7 7th string
	\return Concatenation of the seven strings
*/
#define RETSTRCAT7(s1,s2,s3,s4,s5,s6,s7) static autoserial::typehandler::ConstString tn; if(tn.str==NULL) { \
	tn.str=(char*)malloc(strlen(s1)+strlen(s2)+strlen(s3)+strlen(s4)+strlen(s5)+strlen(s6)+strlen(s7)+1); \
	strcpy(tn.str,s1); strcat(tn.str,s2); strcat(tn.str,s3); strcat(tn.str,s4); strcat(tn.str,s5); strcat(tn.str,s6); strcat(tn.str,s7); \
	} return tn.str
//! Static concatenation of 9 strings
/*! \internal Returns a static string that is a concatenation of the arguments
	\param s1 1st string
	\param s2 2nd string
	\param s3 3rd string
	\param s4 4th string
	\param s5 5th string
	\param s6 6th string
	\param s7 7th string
	\param s8 8th string
	\param s9 9th string
	\return Concatenation of the nine strings
*/
#define RETSTRCAT9(s1,s2,s3,s4,s5,s6,s7,s8,s9) static autoserial::typehandler::ConstString tn; if(tn.str==NULL) { \
	tn.str=(char*)malloc(strlen(s1)+strlen(s2)+strlen(s3)+strlen(s4)+strlen(s5)+strlen(s6)+strlen(s7)+strlen(s8)+strlen(s9)+1); \
	strcpy(tn.str,s1); strcat(tn.str,s2); strcat(tn.str,s3); strcat(tn.str,s4); strcat(tn.str,s5); strcat(tn.str,s6); strcat(tn.str,s7); strcat(tn.str,s8); strcat(tn.str,s9); \
	} return tn.str

		//! Temporary conversion of integer to string
		/*! \internal Creates a string representation of an integer.
		*/
		struct IntToStringConverter
		{
			//! String for storing the conversion result
			char value[16];
			//! Conversion constructor
			/*! \param i Integer to convert
			*/
			IntToStringConverter(int i) { ::sprintf(value,"%d",i); }
		};

		//! Generic type handler
		/*! \internal This is the fallback type handler when no more specialized type
			handler can be found. It assumes the presence of the appropriate read and
			write methods in the type.
		*/
		template<typename x> struct TypeHandler 
		{ 
			//! Get Name of type
			/*! \return Name of type
			*/
			static inline const char *name() { return x::getTypeName(); } 
			//! Write an item of type
			/*! \param s Target stream
				\param item Item to Write
				\return Failure or success code
			*/
			static inline Result write(IOutputStream *s, const x& item) { return item.write(s); }
			//! Read an item of type
			/*! \param s Source stream
				\param item Item to Read
				\return Failure or success code
			*/
			static inline Result read(IInputStream *s, x& item) { return item.read(s); }

			//! Compare two items of type
			/*! \param item1 First member of the comparison
				\param item2 Second member of the comparison
			*/
			static inline void equals(Comparator *v, const x& item1, const x& item2) { item1.compare(v,item2); }
		};

		//! Write an element to the output stream
		/*! \internal This function passes the item to the appropriate type handler. It is 
			used to take advantage of the automatic template parameter selection
			feature of templated functions.
			\param s Target output stream
			\param item Item to Write
			\return Failure or success code
		*/
		template<typename x> Result write(IOutputStream *s, const x& item)
		{
			return TypeHandler<x>::write(s,item);
		}

		//! Read an element from the input stream
		/*! \internal This function passes the item to the appropriate type handler. It is 
			used to take advantage of the automatic template parameter selection
			feature of templated functions.
			\param s Source input stream
			\param item Item to Read
			\return Failure or success code
		*/
		template<typename x> Result read(IInputStream *s, x& item)
		{
			return TypeHandler<x>::read(s,item);
		}

		//! Verifies the equality of two elements of same type
		/*! \internal This function passes the items to the appropriate type handler. It is 
			used to take advantage of the automatic template parameter selection
			feature of templated functions.
			\param item1 First member of the comparison
			\param item2 Second member of the comparison
			\return true if both items are equal
		*/
		template<typename x> void equals(Comparator *v, const x& item1, const x& item2)
		{
			TypeHandler<x>::equals(v,item1,item2);
		}

// There is a type handler for const types, but they cannot be serialized
// or deserialized (obvious, since they are const, they cannot be Read from
// a file...)

// Defines typehandlers for simple types with strict equality
#define SIMPLETYPE(x) template<> struct TypeHandler<x> { \
	static inline const char *name() { return #x; } \
	static inline Result write(IOutputStream *s, const x& item) { return s->write##x(item); } \
	static inline Result writeArray(IOutputStream *s, const x *buf, const int len) { ENTERITEM("array"); Result r; if(AS_FAILED(r=s->writeArray(buf,len,sizeof(x)))) return r; LEAVEITEM("array"); return AS_OK; } \
	static inline Result read(IInputStream *s, x& item) { return s->read##x(item); } \
	static inline Result readArray(IInputStream *s, x *buf, const int len) { ENTERITEM("array"); Result r; if(AS_FAILED(r=s->readArray(buf,len,sizeof(x)))) return r; LEAVEITEM("array"); return AS_OK; } \
	static inline void equals(Comparator *v, const x& item1, const x& item2) { v->memberEqual(item1==item2); } \
	static inline void equalsArray(Comparator *v, const x *buf1, const x *buf2, const int len) { v->memberEqual(memcmp(buf1,buf2,len*sizeof(x))==0); } \
}; \
template<> struct TypeHandler<const x> { \
	static inline const char *name() { return "const "#x; } \
}; \
template<int size> struct TypeHandler<x [size]> {\
	static inline const char *name() { RETSTRCAT4(TypeHandler<x>::name(),"[",IntToStringConverter(size).value,"]"); } \
	static inline Result write(IOutputStream *s, const x (&items) [size]) { ENTERITEM("array"); Result r; if(AS_FAILED(r=s->writeArray(&items,size,sizeof(x)))) return r; LEAVEITEM("array"); return AS_OK; } \
	static inline Result read(IInputStream *s, x (&items) [size]) { ENTERITEM("array"); Result r; if(AS_FAILED(r=s->readArray(&items,size,sizeof(x)))) return r; LEAVEITEM("array"); return AS_OK; } \
	static inline void equals(Comparator *v, const x (&items1) [size], const x (&items2) [size]) { v->memberEqual((memcmp(&items1,&items2,size*sizeof(x))==0) ? true : false); } \
}; \
template<int size> struct TypeHandler<x const [size]> {\
	static inline const char *name() { RETSTRCAT4(TypeHandler<x>::name(),"[",IntToStringConverter(size).value,"]"); } \
	static inline Result write(IOutputStream *s, const x (&items) [size]) { ENTERITEM("array"); Result r; if(AS_FAILED(r=s->writeArray(&items,size,sizeof(x)))) return r; LEAVEITEM("array"); return AS_OK; } \
	static inline Result read(IInputStream *s, x (&items) [size]) { ENTERITEM("array"); Result r; if(AS_FAILED(r=s->readArray(&items,size,sizeof(x)))) return r; LEAVEITEM("array"); return AS_OK; } \
	static inline void equals(Comparator *v, const x (&items1) [size], const x (&items2) [size]) { v->memberEqual((memcmp(&items1,&items2,size*sizeof(x))==0) ? true : false); } \
}

// Defines typehandlers for types with epsilon comparisons
// (comparisons revert to strict if epsilon is 0)
#define SIMPLEFPTYPE(x) template<> struct TypeHandler<x> { \
	static inline const char *name() { return #x; } \
	static inline Result write(IOutputStream *s, const x& item) { return s->write##x(item); } \
	static inline Result writeArray(IOutputStream *s, const x *buf, const int len) { ENTERITEM("array"); Result r; if(AS_FAILED(r=s->writeArray(buf,len,sizeof(x)))) return r; LEAVEITEM("array"); return AS_OK; } \
	static inline Result read(IInputStream *s, x& item) { return s->read##x(item); } \
	static inline Result readArray(IInputStream *s, x *buf, const int len) { ENTERITEM("array"); Result r; if(AS_FAILED(r=s->readArray(buf,len,sizeof(x)))) return r; LEAVEITEM("array"); return AS_OK; } \
	static inline void equals(Comparator *v, const x& item1, const x& item2) { if(v->getEpsilon##x()==0) v->memberEqual(item1==item2); \
																			 else v->memberEqual((item1-item2)<v->getEpsilon##x() && (item2-item1)<v->getEpsilon##x()); } \
	static inline void equalsArray(Comparator *v, const x *buf1, const x *buf2, const int len) { \
			if(v->getEpsilon##x()==0) v->memberEqual(memcmp(buf1,buf2,len*sizeof(x))==0); \
			else { for(int i=0;i<len;++i) { \
				if((buf1[i]-buf2[i])>v->getEpsilon##x() || (buf2[i]-buf1[i])>v->getEpsilon##x()) { v->memberEqual(false); return; } \
			} v->memberEqual(true); } } \
}; \
template<> struct TypeHandler<const x> { \
	static inline const char *name() { return "const "#x; } \
}; \
template<int size> struct TypeHandler<x [size]> {\
	static inline const char *name() { RETSTRCAT4(TypeHandler<x>::name(),"[",IntToStringConverter(size).value,"]"); } \
	static inline Result write(IOutputStream *s, const x (&items) [size]) { ENTERITEM("array"); Result r; if(AS_FAILED(r=s->writeArray(&items,size,sizeof(x)))) return r; LEAVEITEM("array"); return AS_OK; } \
	static inline Result read(IInputStream *s, x (&items) [size]) { ENTERITEM("array"); Result r; if(AS_FAILED(r=s->readArray(&items,size,sizeof(x)))) return r; LEAVEITEM("array"); return AS_OK; } \
	static inline void equals(Comparator *v, const x (&items1) [size], const x (&items2) [size]) { \
		if(v->getEpsilon##x()==0) v->memberEqual((memcmp(&items1,&items2,size*sizeof(x))==0) ? true : false); \
		else { for(int i=0;i<size;++i) { \
			if(((&items1)[i]-(&items2)[i])>v->getEpsilon##x() || ((&items2)[i]-(&items1)[i])>v->getEpsilon##x()) { v->memberEqual(false); return; } \
		} v->memberEqual(true); } } \
}; \
template<int size> struct TypeHandler<x const [size]> {\
	static inline const char *name() { RETSTRCAT4(TypeHandler<x>::name(),"[",IntToStringConverter(size).value,"]"); } \
	static inline Result write(IOutputStream *s, const x (&items) [size]) { ENTERITEM("array"); Result r; if(AS_FAILED(r=s->writeArray(&items,size,sizeof(x)))) return r; LEAVEITEM("array"); return AS_OK; } \
	static inline Result read(IInputStream *s, x (&items) [size]) { ENTERITEM("array"); Result r; if(AS_FAILED(r=s->readArray(&items,size,sizeof(x)))) return r; LEAVEITEM("array"); return AS_OK; } \
	static inline void equals(Comparator *v, const x (&items1) [size], const x (&items2) [size]) { \
		if(v->getEpsilon##x()==0) v->memberEqual((memcmp(&items1,&items2,size*sizeof(x))==0) ? true : false); \
		else { for(int i=0;i<size;++i) { \
			if(((&items1)[i]-(&items2)[i])>v->getEpsilon##x() || ((&items2)[i]-(&items1)[i])>v->getEpsilon##x()) { v->memberEqual(false); return; } \
		} v->memberEqual(true); } } \
}

		//! Type handler for signed 8-bit integers
		SIMPLETYPE(Int8);
		//! Type handler for unsigned 8-bit integers
		SIMPLETYPE(UInt8);
		//! Type handler for signed 16-bit integers
		SIMPLETYPE(Int16);
		//! Type handler for unsigned 16-bit integers
		SIMPLETYPE(UInt16);
		//! Type handler for signed 32-bit integers
		SIMPLETYPE(Int32);
		//! Type handler for unsigned 32-bit integers
		SIMPLETYPE(UInt32);
		//! Type handler for signed 64-bit integers
		SIMPLETYPE(Int64);
		//! Type handler for unsigned 64-bit integers
		SIMPLETYPE(UInt64);
		//! Type handler for booleans
		SIMPLETYPE(Bool);
		
		// Floating-point types use a different macro as an epsilon may be specified
		// for comparisons
		//! Type handler for 32-bit floating point numbers
		SIMPLEFPTYPE(Float);
		//! Type handler for 64-bit floating point numbers
		SIMPLEFPTYPE(Double);

		//! Type handler for STL strings
		template<> struct TypeHandler<std::string> 
		{ 
			//! Get Name of type
			/*! \return Name of type
			*/
			static inline const char *name() { return "std::string"; } 
			static inline Result write(IOutputStream *s, const std::string& item) { return s->writeString(item); } 
			static inline Result read(IInputStream *s, std::string& item) { return s->readString(item); }
			static inline void equals(Comparator *v, const std::string& s1,const std::string& s2) { v->memberEqual(s1==s2); }
		};

		//! Type handler for constant types
		/*! This type handler is required only for completeness, since constant
			types cannot be serialized anyway.
		*/
		template<typename x> struct TypeHandler<const x> 
		{ 
			//! Get Name of type
			/*! \return Name of type
			*/
			static inline const char *name() { RETSTRCAT2("const ",TypeHandler<x>::name()); } 
		};

		//! Type handler for STL allocator
		template<typename x> struct TypeHandler<std::allocator<x> > 
		{
			static inline const char *name() { RETSTRCAT3("std::allocator<",TypeHandler<x>::name(),">"); } 
		};

		//! Type handler for STL less
		template<typename x> struct TypeHandler<std::less<x> > 
		{
			static inline const char *name() { RETSTRCAT3("std::less<",TypeHandler<x>::name(),">"); } 
		};

		//! Type handler for STL pair
		template<typename a, typename b> struct TypeHandler<std::pair<a,b> > 
		{ 
			static inline const char *name() { RETSTRCAT5("std::pair<",TypeHandler<a>::name(),",",TypeHandler<b>::name(),">"); } 
			static inline Result write(IOutputStream *s, const std::pair<a,b>& item) 
			{ 
				ENTERITEM("first");
				if(AS_FAILED(typehandler::write(s,item.first)))
					return AS_FAIL;
				LEAVEITEM("first");
				ENTERITEM("second");
				if(AS_FAILED(typehandler::write(s,item.second)))
					return AS_FAIL;
				LEAVEITEM("second");
				return AS_OK;
			}
			static inline Result read(IInputStream *s, std::pair<a,b>& item) 
			{ 
				ENTERITEM("first");
				if(AS_FAILED(typehandler::read(s,item.first)))
					return AS_FAIL;
				LEAVEITEM("first");
				ENTERITEM("second");
				if(AS_FAILED(typehandler::read(s,item.second)))
					return AS_FAIL;
				LEAVEITEM("second");
				return AS_OK;
			}

			static inline void equals(Comparator *v, const std::pair<a,b>& item1, const std::pair<a,b>& item2)
			{
				v->enterMember("first");
				typehandler::equals(v,item1.first,item2.first);
				v->leaveMember("first");
				v->enterMember("second");
				typehandler::equals(v,item1.second,item2.second);
				v->leaveMember("second");
			}
		};

		//! Type handler for STL vector
		template<typename x, typename a> struct TypeHandler<std::vector<x,a> > 
		{ 
			static inline const char *name() { RETSTRCAT5("std::vector<",TypeHandler<x>::name(),",",TypeHandler<a>::name(),">"); } 
			static inline Result write(IOutputStream *s, const std::vector<x,a>& item) 
			{ 
				ENTERITEM("size");
				XPSize size=(XPSize)item.size();
				if(AS_FAILED(typehandler::write(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(typename std::vector<x,a>::const_iterator i=item.begin();i!=item.end();i++)
				{
					ENTERITEM("element");
					if(AS_FAILED(typehandler::write(s,*i)))
						return AS_FAIL;
					LEAVEITEM("element");
				}
				return AS_OK;
			}
			static inline Result read(IInputStream *s, std::vector<x,a>& item) 
			{ 
				item.clear();
				ENTERITEM("size");
				XPSize size;
				if(AS_FAILED(typehandler::read(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(Size i=0;i<size;i++)
				{
					ENTERITEM("element");
					x t;
					if(AS_FAILED(typehandler::read(s,t)))
						return AS_FAIL;
					item.push_back(t);
					LEAVEITEM("element");
				}
				return AS_OK;
			} 

			static inline void equals(Comparator *v, const std::vector<x,a>& item1, const std::vector<x,a>& item2)
			{
				if(item1.size()!=item2.size())
				{
					v->enterMember("size");
					v->memberEqual(false);
					v->leaveMember("size");
					return;
				}

				v->enterMember("size");
				v->memberEqual(true);
				v->leaveMember("size");
				typename std::vector<x,a>::const_iterator i1=item1.begin();
				typename std::vector<x,a>::const_iterator i2=item2.begin();
				while(i1!=item1.end())
				{
					v->enterMember("item");
					typehandler::equals(v,*i1,*i2);
					v->leaveMember("item");
					++i1;
					++i2;
				}
			}
		};

		//! Type handler for STL deque
		template<typename x, typename a> struct TypeHandler<std::deque<x,a> > 
		{ 
			static inline const char *name() { RETSTRCAT5("std::deque<",TypeHandler<x>::name(),",",TypeHandler<a>::name(),">"); } 
			static inline Result write(IOutputStream *s, const std::deque<x,a>& item) 
			{ 
				ENTERITEM("size");
				XPSize size=(XPSize)item.size();
				if(AS_FAILED(typehandler::write(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(typename std::deque<x,a>::const_iterator i=item.begin();i!=item.end();i++)
				{
					ENTERITEM("element");
					if(AS_FAILED(typehandler::write(s,*i)))
						return AS_FAIL;
					LEAVEITEM("element");
				}
				return AS_OK;
			}
			static inline Result Read(IInputStream *s, std::deque<x,a>& item) 
			{ 
				item.clear();
				ENTERITEM("size");
				XPSize size;
				if(AS_FAILED(typehandler::read(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(Size i=0;i<size;i++)
				{
					ENTERITEM("element");
					x t;
					if(AS_FAILED(typehandler::read(s,t)))
						return AS_FAIL;
					item.push_back(t);
					LEAVEITEM("element");
				}
				return AS_OK;
			}

			static inline void equals(Comparator *v, const std::deque<x,a>& item1, const std::deque<x,a>& item2)
			{ 
				if(item1.size()!=item2.size())
				{
					v->enterMember("size");
					v->memberEqual(false);
					v->leaveMember("size");
					return;
				}

				v->enterMember("size");
				v->memberEqual(true);
				v->leaveMember("size");
				typename std::deque<x,a>::const_iterator i1=item1.begin();
				typename std::deque<x,a>::const_iterator i2=item2.begin();
				while(i1!=item1.end())
				{
					v->enterMember("item");
					typehandler::equals(v,*i1,*i2);
					v->leaveMember("item");
					++i1;
					++i2;
				}
			}
		};

		//! Type handler for STL list
		template<typename x, typename a> struct TypeHandler<std::list<x,a> > 
		{ 
			static inline const char *name() { RETSTRCAT5("std::list<",TypeHandler<x>::name(),",",TypeHandler<a>::name(),">"); } 
			static inline Result write(IOutputStream *s, const std::list<x,a>& item) 
			{ 
				ENTERITEM("size");
				XPSize size=(XPSize)item.size();
				if(AS_FAILED(typehandler::write(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(typename std::list<x,a>::const_iterator i=item.begin();i!=item.end();i++)
				{
					ENTERITEM("element");
					if(AS_FAILED(typehandler::write(s,*i)))
						return AS_FAIL;
					LEAVEITEM("element");
				}
				return AS_OK;
			}
			static inline Result read(IInputStream *s, std::list<x,a>& item) 
			{ 
				item.clear();
				ENTERITEM("size");
				XPSize size;
				if(AS_FAILED(typehandler::read(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(Size i=0;i<size;i++)
				{
					ENTERITEM("element");
					x t;
					if(AS_FAILED(typehandler::read(s,t)))
						return AS_FAIL;
					item.push_back(t);
					LEAVEITEM("element");
				}
				return AS_OK;
			}

			static inline void equals(Comparator *v, const std::list<x,a>& item1, const std::list<x,a>& item2)
			{ 
				if(item1.size()!=item2.size())
				{
					v->enterMember("size");
					v->memberEqual(false);
					v->leaveMember("size");
					return;
				}

				v->enterMember("size");
				v->memberEqual(true);
				v->leaveMember("size");
				typename std::list<x,a>::const_iterator i1=item1.begin();
				typename std::list<x,a>::const_iterator i2=item2.begin();
				while(i1!=item1.end())
				{
					v->enterMember("item");
					typehandler::equals(v,*i1,*i2);
					v->leaveMember("item");
					++i1;
					++i2;
				}
			}
		};

		//! Type handler for STL set
		template<typename x, typename t, typename a> struct TypeHandler<std::set<x,t,a> > 
		{ 
			static inline const char *name() { RETSTRCAT7("std::set<",TypeHandler<x>::name(),",",TypeHandler<t>::name(),",",TypeHandler<a>::name(),">"); } 
			static inline Result write(IOutputStream *s, const std::set<x,t,a>& item) 
			{ 
				ENTERITEM("size");
				XPSize size=(XPSize)item.size();
				if(AS_FAILED(typehandler::write(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(typename std::set<x,t,a>::const_iterator i=item.begin();i!=item.end();i++)
				{
					ENTERITEM("element");
					if(AS_FAILED(typehandler::write(s,*i)))
						return AS_FAIL;
					LEAVEITEM("element");
				}
				return AS_OK;
			}
			static inline Result Read(IInputStream *s, std::set<x,t,a>& item) 
			{ 
				item.clear();
				ENTERITEM("size");
				XPSize size;
				if(AS_FAILED(typehandler::read(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(Size i=0;i<size;i++)
				{
					ENTERITEM("element");
					x tm;
					if(AS_FAILED(typehandler::read(s,tm)))
						return AS_FAIL;
					item.insert(tm);
					LEAVEITEM("element");
				}
				return AS_OK;
			}

			static inline void equals(Comparator *v, const std::set<x,t,a>& item1, const std::set<x,t,a>& item2)
			{ 
				if(item1.size()!=item2.size())
				{
					v->enterMember("size");
					v->memberEqual(false);
					v->leaveMember("size");
					return;
				}

				v->enterMember("size");
				v->memberEqual(true);
				v->leaveMember("size");
				typename std::set<x,t,a>::const_iterator i1=item1.begin();
				typename std::set<x,t,a>::const_iterator i2=item2.begin();
				while(i1!=item1.end())
				{
					v->enterMember("item");
					typehandler::equals(v,*i1,*i2);
					v->leaveMember("item");
					++i1;
					++i2;
				}
			}
		};

		//! Type handler for STL multiset
		template<typename x, typename t, typename a> struct TypeHandler<std::multiset<x,t,a> > 
		{ 
			static inline const char *name() { RETSTRCAT7("std::multiset<",TypeHandler<x>::name(),",",TypeHandler<t>::name(),",",TypeHandler<a>::name(),">"); } 
			static inline Result write(IOutputStream *s, const std::multiset<x,t,a>& item) 
			{ 
				ENTERITEM("size");
				XPSize size=(XPSize)item.size();
				if(AS_FAILED(typehandler::write(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(typename std::multiset<x,t,a>::const_iterator i=item.begin();i!=item.end();i++)
				{
					ENTERITEM("element");
					if(AS_FAILED(typehandler::write(s,*i)))
						return AS_FAIL;
					LEAVEITEM("element");
				}
				return AS_OK;
			}
			static inline Result Read(IInputStream *s, std::multiset<x,t,a>& item) 
			{ 
				item.clear();
				ENTERITEM("size");
				XPSize size;
				if(AS_FAILED(typehandler::read(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(Size i=0;i<size;i++)
				{
					ENTERITEM("element");
					x tm;
					if(AS_FAILED(typehandler::read(s,tm)))
						return AS_FAIL;
					item.insert(tm);
					LEAVEITEM("element");
				}
				return AS_OK;
			}

			static inline void equals(Comparator *v, const std::multiset<x,t,a>& item1, const std::multiset<x,t,a>& item2)
			{ 
				if(item1.size()!=item2.size())
				{
					v->enterMember("size");
					v->memberEqual(false);
					v->leaveMember("size");
					return;
				}

				v->enterMember("size");
				v->memberEqual(true);
				v->leaveMember("size");
				typename std::multiset<x,t,a>::const_iterator i1=item1.begin();
				typename std::multiset<x,t,a>::const_iterator i2=item2.begin();
				while(i1!=item1.end())
				{
					v->enterMember("item");
					typehandler::equals(v,*i1,*i2);
					v->leaveMember("item");
					++i1;
					++i2;
				}
			}
		};

		//! Type handler for STL map
		template<typename x, typename y, typename t, typename a> struct TypeHandler<std::map<x,y,t,a> > 
		{ 
			static inline const char *name() { RETSTRCAT9("std::map<",TypeHandler<x>::name(),",",TypeHandler<y>::name(),",",TypeHandler<t>::name(),",",TypeHandler<a>::name(),">"); } 
			static inline Result write(IOutputStream *s, const std::map<x,y,t,a>& item) 
			{ 
				ENTERITEM("size");
				XPSize size=(XPSize)item.size();
				if(AS_FAILED(typehandler::write(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(typename std::map<x,y,t,a>::const_iterator i=item.begin();i!=item.end();i++)
				{
					ENTERITEM("element");
					if(AS_FAILED(typehandler::write(s,*i)))
						return AS_FAIL;
					LEAVEITEM("element");
				}
				return AS_OK;
			}
			static inline Result read(IInputStream *s, std::map<x,y,t,a>& item) 
			{ 
				item.clear();
				ENTERITEM("size");
				XPSize size;
				if(AS_FAILED(typehandler::read(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(Size i=0;i<size;i++)
				{
					ENTERITEM("element");
					std::pair<x,y> tm;
					if(AS_FAILED(typehandler::read(s,tm)))
						return AS_FAIL;
					item.insert(tm);
					LEAVEITEM("element");
				}
				return AS_OK;
			}

			static inline void equals(Comparator *v, const std::map<x,y,t,a>& item1, const std::map<x,y,t,a>& item2)
			{ 
				if(item1.size()!=item2.size())
				{
					v->enterMember("size");
					v->memberEqual(false);
					v->leaveMember("size");
					return;
				}

				v->enterMember("size");
				v->memberEqual(true);
				v->leaveMember("size");
				typename std::map<x,y,t,a>::const_iterator i1=item1.begin();
				typename std::map<x,y,t,a>::const_iterator i2=item2.begin();
				while(i1!=item1.end())
				{
					v->enterMember("item");
					typehandler::equals(v,*i1,*i2);
					v->leaveMember("item");
					++i1;
					++i2;
				}
			}
		};

		//! Type handler for STL multimap
		template<typename x, typename y, typename t, typename a> struct TypeHandler<std::multimap<x,y,t,a> > 
		{ 
			static inline const char *name() { RETSTRCAT9("std::multimap<",TypeHandler<x>::name(),",",TypeHandler<y>::name(),",",TypeHandler<t>::name(),",",TypeHandler<a>::name(),">"); } 
			static inline Result write(IOutputStream *s, const std::multimap<x,y,t,a>& item) 
			{ 
				ENTERITEM("size");
				XPSize size=(XPSize)item.size();
				if(AS_FAILED(typehandler::write(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(typename std::multimap<x,y,t,a>::const_iterator i=item.begin();i!=item.end();i++)
				{
					ENTERITEM("element");
					if(AS_FAILED(typehandler::write(s,*i)))
						return AS_FAIL;
					LEAVEITEM("element");
				}
				return AS_OK;
			}
			static inline Result read(IInputStream *s, std::multimap<x,y,t,a>& item) 
			{ 
				item.clear();
				ENTERITEM("size");
				XPSize size;
				if(AS_FAILED(typehandler::read(s,size)))
					return AS_FAIL;
				LEAVEITEM("size");
				for(Size i=0;i<size;i++)
				{
					ENTERITEM("element");
					std::pair<x,y> tm;
					if(AS_FAILED(typehandler::read(s,tm)))
						return AS_FAIL;
					item.insert(tm);
					LEAVEITEM("element");
				}
				return AS_OK;
			}

			static inline void equals(Comparator *v, const std::multimap<x,y,t,a>& item1, const std::multimap<x,y,t,a>& item2)
			{ 
				if(item1.size()!=item2.size())
				{
					v->enterMember("size");
					v->memberEqual(false);
					v->leaveMember("size");
					return;
				}

				v->enterMember("size");
				v->memberEqual(true);
				v->leaveMember("size");
				typename std::multimap<x,y,t,a>::const_iterator i1=item1.begin();
				typename std::multimap<x,y,t,a>::const_iterator i2=item2.begin();
				while(i1!=item1.end())
				{
					v->enterMember("item");
					typehandler::equals(v,*i1,*i2);
					v->leaveMember("item");
					++i1;
					++i2;
				}
			}
		};


		//! Type handler for fixed-size arrays
		template<typename x, int size> struct TypeHandler<x [size]>
		{
			static inline const char *name() { RETSTRCAT4(TypeHandler<x>::name(),"[",IntToStringConverter(size).value,"]"); } 
			static inline Result write(IOutputStream *s, const x (&items) [size]) 
			{
				for(Size i=0;i<size;i++)
				{
					ENTERITEM("element");
					if(AS_FAILED(typehandler::write(s,items[i])))
						return AS_FAIL;
					LEAVEITEM("element");
				}
				return AS_OK;
			}
			static inline Result read(IInputStream *s, x (&items) [size]) 
			{ 
				for(Size i=0;i<size;i++)
				{
					ENTERITEM("element");
					if(AS_FAILED(typehandler::read(s,items[i])))
						return AS_FAIL;
					LEAVEITEM("element");
				}
				return AS_OK;
			}

			static inline void equals(Comparator *v, const x (&items1) [size], const x (&items2) [size])
			{
				for(Size i=0;i<size;i++)
				{
					v->enterMember("item");
					typehandler::equals(v,items1[i],items2[i]);
					v->leaveMember("item");
				}
			}
		};

		//! Writing, reading and comparison functions for dynamic arrays
		template<typename x> Result writeArray(IOutputStream *s, const x* buf, const int len)
		{
			if(len!=0)
				return TypeHandler<x>::writeArray(s,buf,len);
			return AS_OK;
		}
		template<typename x> Result readArray(IInputStream *s, x*& buf, const int len)
		{
			if(len!=0)
			{
				buf=(x*)malloc(len*sizeof(x));
				return TypeHandler<x>::readArray(s,buf,len);
			}
			return AS_OK;
		}
		template<typename x> void equalsArray(Comparator *v, const x* buf1, const int len1, const x* buf2, const int len2)
		{
			if(len1!=len2)
				v->memberEqual(false);
			else
				TypeHandler<x>::equalsArray(v,buf1,buf2,len1);
		}

	}
}

#endif

