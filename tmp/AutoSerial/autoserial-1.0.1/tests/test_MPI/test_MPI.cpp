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

#include <iostream>

#include "../objects.h"

#undef SEEK_SET
#undef SEEK_CUR
#undef SEEK_END

//#define AS_SYS_TYPES
#include <autoserial/autoserial_mpi.h>

using namespace autoserial;

int main(int argc, char *argv[])
{
	MPI_Init(&argc,&argv);
	int rank, size;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Comm_size(MPI_COMM_WORLD, &size);

	if (size < 3)
	{
		std::cout << "Test must be run with at least 3 processes" << std::endl;
		MPI_Finalize();
		return 1;
	}

	// C bindings

	// 0 sends an object to 1 and back
	if (rank == 0)
	{
		ListObject l1;
		l1.init();
		if (AS_FAILED(AS_MPI_Send(l1, 1, 0, MPI_COMM_WORLD)))
		{
			std::cout << "Process 0 could not send object using AS_MPI_Send" << std::endl;
			return 1;
		}
		ListObject *l2;
		MPI_Status status;
		if (AS_FAILED(AS_MPI_Recv((ISerializable*&)l2, 1, 0, MPI_COMM_WORLD, &status)))
		{
			std::cout << "Process 0 could not receive object using AS_MPI_Recv" << std::endl;
			return 1;
		}
		// Compare objects
		Comparator c;
		c.printMembersEquality(&l1, l2, 2);
		delete l2;
	}
	else if (rank == 1)
	{
		ListObject *l1;
		MPI_Status status;
		if (AS_FAILED(AS_MPI_Recv((ISerializable*&)l1, 0, 0, MPI_COMM_WORLD, &status)))
		{
			std::cout << "Process 1 could not receive object using AS_MPI_Recv" << std::endl;
			return 1;
		}
		if (AS_FAILED(AS_MPI_Send(*l1, 0, 0, MPI_COMM_WORLD)))
		{
			std::cout << "Process 0 could not send object using AS_MPI_Send" << std::endl;
			return 1;
		}
		delete l1;
	}

	// C++ bindings

	// Build communicator
	AutoserialCommunicator asComm(MPI_COMM_WORLD);
	if (rank != asComm.Get_rank())
	{
		std::cout << "Ranks in autoserialCommunicator and MPI_COMM_WORLD are different" << std::endl;
		return 1;
	}

	if (rank==0)
	{
		ListObject l1;
		l1.init();

		if (AS_FAILED(asComm.Send(l1, 1, 0)))
		{
			std::cout << "Could not send " << l1.getTypeName() << " to process 1" << std::endl;
			return 1;
		}

		MPI::Status st;
		ListObject *l2 = (ListObject*)asComm.Recv(MPI_ANY_SOURCE, 0, st);
		if (l2 == NULL)
		{
			std::cout << "Could not receive " << l2->getTypeName() << " from process 1" << std::endl;
			return 1;
		}
		std::cout << "Received object from " << st.Get_source() << std::endl;

		// Compare objects
		Comparator c;
		c.printMembersEquality(&l1, l2, 2);
		delete l2;
	}
	else if (rank==1)
	{
		ListObject *l1;
		l1 = asComm.Recv<ListObject>(0, 0);
		if (l1 == NULL)
		{
			std::cout << "Could not recv ListObject from process 0" << std::endl;
			return 1;
		}

		if (AS_FAILED(asComm.Send(*l1, 0, 0)))
		{
			std::cout << "Process 1 could not send object back to 0" << std::endl;
			return 1;
		}

		delete l1;
	}



	/* This example uses templated functions of AutoserialCommunicator
	This function must run in 3 processes of rank 0, 1 and 2 in MPI_COMM_WORLD
	Process rank 0 and 2 send ListObject to process rank 1
	*/
/*void runAS_MPI2(bool print=true) {
	AS_Comm as_comm(MPI_COMM_WORLD);
	int rank=as_comm.Get_rank();

	if (as_comm.Get_size()<3)
	{
		std::cout << "Need at least 3 processes (mpirun -np 3)" << std::endl;
		return;
	}

	if (rank==0 || rank==2)
	{
		int dest=1; // Destination process
		SimpleObject so;

		// Just distinguish process source by the contents of objects
		so.item1=100+rank;
		so.item2=200.0+rank;

		if AS_FAILED(as_comm.Send(so,dest))
		{
			std::cout << "Could not send " << so.getTypeName() << " to process " << dest << std::endl;
			return;
		}

		std::cout << "Successful sending of " << so.getTypeName() << " to process " << dest << std::endl;
		if (print) 
			so.print();

	}
	else if (rank==1)
	{
		SimpleObject *so1;
		SimpleObject *so2;

		// Receive data from any processes ...
		if ((so1=as_comm.Recv<SimpleObject>(MPI_ANY_SOURCE))==NULL)
		{
			std::cout << "Could not recv SimpleObject!" << std::endl;
			return;
		}
		std::cout << "Successful recv " << so1->getTypeName() << std::endl;
		if (print)
			so1->print();

		if ((so2=as_comm.Recv<SimpleObject>(MPI_ANY_SOURCE))==NULL) 
		{
			std::cout << "Could not recv SimpleObject!" << std::endl;
			return;
		}

		std::cout << "Successful recv " << so2->getTypeName() << std::endl;
		if (print)
			so2->print();

		delete so1;
		delete so2;
	}
}*/

	MPI_Finalize();

	return 0;
}
