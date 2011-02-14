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


#include "../include/autoserial/defs.h"
#include "../include/autoserial/base.h"
#include "../include/autoserial/comparator.h"

namespace autoserial
{

	class Member
	{
	public:
		const char *name; // This is used only for printing purposes
		bool equal;
		std::vector<Member*> members;
		Member *ref;

		Member(const char *name) { this->name=name; equal=true; ref=NULL; }

		~Member()
		{
			for(std::vector<Member *>::const_iterator it=members.begin();it!=members.end();++it)
				delete (*it);
		}

		void print(int currentDepth,int maxDepth)
		{
			if(maxDepth!=-1&&currentDepth>maxDepth)
				return;
			for(int i=0;i<currentDepth;++i)
				printf("  ");
			if(ref==NULL)
			{
				(equal) ? printf("%s: equal\n",name) : printf("%s: modified\n",name);
				for(Size i=0;i<members.size();++i)
					members[i]->print(currentDepth+1,maxDepth);
			}
			else
				(ref->equal) ? printf("%s: referenced %s equal\n",name,ref->name) : printf("%s: referenced %s modified\n",name,ref->name);
		}

		// Updates equality state in member tree. Returns true if a member was updated
		bool verifyEquality()
		{
			bool memberEquals=true;
			bool updated=false;
			for(std::vector<Member *>::const_iterator it=members.begin();it!=members.end();++it)
			{
				updated=(*it)->verifyEquality() && updated;
				memberEquals=(*it)->equal && memberEquals;
			}

			if(equal && !memberEquals)
			{
				equal=false;
				updated=true;
			}
			return updated;
		}
	};

	void Comparator::enterMember(const char *name)
	{
		// Verify that name is unique within members
		// Update: STL containers now uses item for every element
		/*Member *parent=currentMembers.back();
		for(Size i=0;i<parent->members.size();++i)
		{
			if(strcmp(parent->members[i]->name,name)==0)
				verifLog.write(0) << "!! WARNING !! Several members are named " << name << ", please check your custom equalsV() code !!";
		}*/
		Member *m=new Member(name);

		currentMembers.push_back(m);
	}

	void Comparator::leaveMember(const char *name)
	{
		Member *m=currentMembers.back();
		if(strcmp(m->name,name)!=0)
		{
			//std::cout << "!! WARNING !! leaveMember(\"" << name << "\") does not match enterMember(\"" << m->name << "\"), please check your custom equalsV() code !!";
		}
		currentMembers.pop_back();
		Member *parent=currentMembers.back();
		parent->equal=m->equal && parent->equal;
		parent->members.push_back(m);
	}

	void Comparator::memberEqual(bool equal)
	{
		currentMembers.back()->equal=(equal && currentMembers.back()->equal);
	}

	bool Comparator::compare(const ISerializable *item1, const ISerializable *item2, std::map<std::string,bool> *accessMap, bool print, int maxDepth)
	{
		assert((item1!=NULL) && (item2!=NULL));

		// Setup
		Member head(item1->getTypeNameV());
		currentMembers.push_back(&head);

		refs1.clear();
		refs2.clear();

		refs1.push_back(item1);
		refs2.push_back(item2);
		referencedMembers[0]=&head;

		// Add first result
		refs1[0]->compareV(this,refs2[0]);
		currentMembers.pop_back();
		if(currentMembers.size()!=0)
			incorrectEqualityCheck(refs1[0]->getTypeNameV());

		Size oldSize=1;
		while(oldSize!=refs1.size())
		{
			Size newSize=refs1.size();
			// Loop is entered if areEqualRef below was called and a new ref was added
			for(Size i=oldSize;i<newSize;i++)
			{
                currentMembers.push_back(referencedMembers[(int)i]);
				refs1[i]->compareV(this,refs2[i]);
				currentMembers.pop_back();
				if(currentMembers.size()!=0)
					incorrectEqualityCheck(refs1[0]->getTypeNameV());
			}
			oldSize=newSize;
		}

		// The object structure has been reconstructed, so we must propagate modifications to the head
		bool updated;
		do
		{
			updated=false;
			for(std::vector<Member *>::const_iterator it=head.members.begin();it!=head.members.end();++it)
			{
				if((*it)->equal)
					updated=(*it)->verifyEquality() || updated; // This updates (*it)->equal
				head.equal=(*it)->equal && head.equal;
				if (accessMap!=NULL)
					(*accessMap)[(*it)->name]=!(*it)->equal;
			}
		}
		while(updated);

		if(print)
			head.print(0,maxDepth);

		// Cleanup
		referencedMembers.clear();
		return head.equal;
	}

	bool Comparator::areEqual(const ISerializable *item1, const ISerializable *item2)
	{
		return compare(item1,item2,NULL,false,0);
	}

	std::map<std::string,bool> Comparator::getEqualityMap(const ISerializable *item1, const ISerializable *item2)
	{
		std::map<std::string,bool> accessMap;
		compare(item1,item2,&accessMap,false,0);
		return accessMap;
	}

	void Comparator::printMembersEquality(const ISerializable *item1, const ISerializable *item2, int maxDepth)
	{
		compare(item1,item2,NULL,true,maxDepth);
	}

	//! Comparison function for pointed objects
	void Comparator::areEqualRef(const autoserial::ISerializable *item1, const autoserial::ISerializable *item2)
	{
		assert(refs1.size()==refs2.size());

		// Special case for NULL pointers
		if(item1==NULL || item2==NULL)
		{
			if(item1==item2)
				memberEqual(true);
			else
				memberEqual(false);
			return;
		}

		for(Size i=0;i<refs1.size();i++)
		{
			if(refs1[i]==item1)
			{
				assert(refs2[i]==item2);
				// Current member points to referenced member
				currentMembers.back()->ref=referencedMembers[(int)i];
				return;
			}
		}
		// Items are seen for the first time
		refs1.push_back(item1);
		refs2.push_back(item2);
		// Keep track of parent so that it can be updated later
		referencedMembers[(int)refs1.size()-1]=currentMembers.back();
	}

	// Prototype is in include/base.h
	bool ISerializable::equals(const ISerializable *s) const
	{
		Comparator c;
		return c.areEqual(this,s);
	};
}

