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

/*! \file buffer.h
	\brief Buffer container classes

	A class for typed resizable memory blocks.
*/

#ifndef INCLUDED_BUFFER_H
#define INCLUDED_BUFFER_H

#include "base.h"
#include "classfactory.h"
#include "typehandler.h"

namespace autoserial
{
	namespace internal
	{

		//! Resizeable Memory block of a simple type
		/*! This class provides a wrapper around memory blocks containing items of
			a specific type. For instance, the following can be replaced:
			<br><pre>Int32 *data=new Int32[itemCount];</pre>
			<br>by
			<br><pre>Buffer<Int32> data(itemCount);</pre>
			<br>The buffer overloads standard cast operators, so its use should be
			fairly transparent when a traditional allocation is replaced.
			<p>This class can be serialized.
		*/
		template<typename bt> class Buffer : public ISerializable
		{
		private:
			NOBASECLASSES;
			AS_TYPENAMETEMPLATE1(Buffer,bt) 
				AS_SERIALVIRTUAL(Buffer) 
				AS_IDENTIFYSTATIC(Buffer) 
				AS_REGISTERCLASS(Buffer) 

		private:
			//! Pointer to data
			bt *data;
			//! Number of items in memory block
			Size len;

		public:
			//! Constructor for a zero-length memory block
			Buffer() { data=NULL; len=0; }

			//! Constructor for a memory block
			/*! \param l Initial size of memory block
			*/
			Buffer(Size l) { data=(bt*)malloc(l*sizeof(bt)); len=l; }

			//! Constructor for a memory block
			/*! \param src Pointer to data to copy to memory block
				\param l   Size of memory block pointed by src
				*/
			Buffer(const bt *src, Size l) 
			{ 
				data=NULL; 
				len=0; 
				if(src && l) 
				{ 
					data=(bt*)malloc(l*sizeof(bt)); 
					memcpy(data,src,l*sizeof(bt)); 
					len=l; 
				} 
			}


			//! Copy constructor for a memory block
			/*! \param src Memory block to copy data from
			*/
			Buffer(const Buffer &src) 
			{ 
				data=NULL; 
				len=0; 
				if(src.data) 
				{ 
					data=(bt*)malloc(src.len*sizeof(bt)); 
					memcpy(data,src.data,src.len*sizeof(bt)); 
					len=src.len; 
				} 
			}

			//! Destructor
			~Buffer() { if(data) free(data); }

			//! Get size of memory block
			Size size() const { return len; }

			//! Copy a memory block into this one
			/*! \param src Source memory block
			*/
			Buffer& operator=(const Buffer& src) 
			{ 
				if(data) 
					free(data); 
				data=NULL; 
				len=0; 
				if(src.data) 
				{ 
					data=(bt*)malloc(src.len*sizeof(bt)); 
					assert(data!=NULL);
					memcpy(data,src.data,src.len*sizeof(bt)); 
					len=src.len; 
				} 
				return *this; 
			}

			//! Cast to data type pointer
			operator bt *() { return data; }

			//! Get pointer to data
			bt *get() const { return data; }

			//	bt& operator[](Int32 i) { return ((bt*)data)[i]; }
			//! Resize memory block
			/*! \param newSize New size of memory block
			*/
			void resize(Size newSize) 
			{ 
				if(newSize)
				{
					if(data)
						data=(bt*)realloc(data,newSize*sizeof(bt));
					else
						data=(bt*)malloc(newSize*sizeof(bt));
					Size oldSize = len;
					len=newSize;

					// Allocation must succeed
					assert(data!=NULL);

					if(newSize>oldSize)
					{
						// Reset content of newly allocated memory to guarantee
						// identical memory content during state comparisons
						memset(data+oldSize,0,(newSize-oldSize)*sizeof(bt));
					}
				}
				else
				{
					if(data)
						free(data);
					len=0;
					data=NULL;
				} 
			}

			void compare(Comparator *v,const Buffer<bt>& item) const
			{
				if(len==item.len)
				{
					v->enterMember("size");
					v->memberEqual(true);
					v->leaveMember("size");
					v->enterMember("data");
					v->memberEqual(memcmp(data,item.data,len*sizeof(bt))==0);
					v->leaveMember("data");
				}
				else
				{
					v->enterMember("size");
					v->memberEqual(false);
					v->leaveMember("size");
					v->enterMember("data");
					v->memberEqual(false);
					v->leaveMember("data");
				}
			}

			//! Serialize memory block
			Result write(IOutputStream *s) const
			{
				Result r;
				ENTERITEM("size");
				XPSize tl=(XPSize)len;
				if(AS_FAILED(r=typehandler::write(s,tl)))
					return r;	//return AS_FAIL;
				LEAVEITEM("size");
				ENTERITEM("data");
				if(AS_FAILED(r=s->writeArray(data,len,sizeof(bt))))
					return r; // return AS_FAIL;
				LEAVEITEM("data");
				return AS_OK;
			}

			//! Deserialize memory block
			Result read(IInputStream *s)
			{
				Result r;
				if(data)
					free(data);
				data=NULL;
				len=0;
				ENTERITEM("size");
				XPSize rlen;
				if(AS_FAILED(r=typehandler::read(s,rlen)))
					return r;   //return AS_FAIL;
				LEAVEITEM("size");
				len=rlen;
				if(len) 
				{ 
					ENTERITEM("data");
					data=(bt*)malloc(len*sizeof(bt)); 
					if(AS_FAILED(r=s->readArray(data,len,sizeof(bt))))
						return r;  // return AS_FAIL;
					LEAVEITEM("data");
				} 
				return AS_OK;
			}
		};
	}
}

#endif

