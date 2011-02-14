Linux/Unix/Mac OS X/Cygwin
==========================

The Linux/Unix/Mac OS X/Cygwin version uses make, and requires g++ version 3
or greater. It also works with the Intel compiler.

It requires no external libraries.

Compile with 'make'. This produces 'libautoserial.a' that you can link against your program.

Compile and run the test programs with 'make test'.

Compile and run the test MPI program with 'make testMPI'. The makefile assumes that
'mpicXX' and 'mpiexec' are available.




Windows
=======

The archives include project files for Visual Studio 2005. The code also
works with Visual Studio .NET 2003.

It requires no external libraries.

The 'test_MPI' program can be compiled and run from the command line or using
the Microsoft Compute Cluster SDK, which includes an MPI library and integrates
MPI applications within the Visual Studio debugger.
