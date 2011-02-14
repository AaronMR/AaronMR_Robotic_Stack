http://cs.baylor.edu/~donahoo/practical/CSockets/practical/


Practical C++ Sockets -- Unix/Windows compatible

Compilation Instructions:

Linux:  Make sure to include the pthread library when compiling the
threaded server example.

Windows:  When you compile this program on Windows, make sure you add
wsock32.lib to the library module list (Project->Setting->Link under
VC++).

TCPEchoServer-Thread.cpp requires a PThreads library to run under
Windows.  Here's how I got this to work using the pthreads-win32
library in VC++:

1.  Download the latest snapshot of pthread-win32 from
    http://sources.redhat.com/pthreads-win32.  You probably want the
    precompiled version named pthreads-yyyy-mm-dd.exe.
2.  Copy the include files pthread.h and sched.h to some appropriate
    spot.  If the compiler does not already know this spot, tell it
    about it.  (I put the files in the same directory as the source
    and included . in the "Additional include directories" list
    (Under VC++ 6.0:  Project->Settings, C/C++ tab, select
    "Preprocessor" category).
3.  Add the correct library to the library list (Under VC++ 6.0:
    Project->Settings, Link tab, select "General" category, ).  I used
    pthreadVC.lib (V = VC++, C = C-style exception handling).  You
    select the pthread library based on the behavior you want and
    compiler you are using.  Read the pthreads-win32 readme for
    details.
4.  Finish setting up your project.
5.  Copy the DLL to the appropriate spot, such as somewhere in the path
    or in the same directory as the executable.
