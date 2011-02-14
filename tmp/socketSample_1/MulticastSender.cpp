/*
 *   C++ sockets on Unix and Windows
 *   Copyright (C) 2002
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "PracticalSocket.h"  // For UDPSocket and SocketException
#include <iostream>           // For cout and cerr
#include <cstdlib>            // For atoi()

#ifdef WIN32
#include <windows.h>          // For ::Sleep()
void sleep(unsigned int seconds) {::Sleep(seconds * 1000);}
#else
#include <unistd.h>           // For sleep()
#endif


using namespace std;

int main(int argc, char *argv[]) {
  if ((argc < 4) || (argc > 5)) {   // Test for correct number of arguments
    cerr << "Usage: " << argv[0] 
         << " <Destination Address> <Destination Port> <Send String> [<TTL>]\n";
    exit(1);
  }

  string servAddress = argv[1];         // First arg: multicast address
  unsigned short port = atoi(argv[2]);  // Second arg: port
  char* sendString = argv[3];           // Third arg: string to echo

  unsigned char multicastTTL = 1;       // Default TTL
  if (argc == 5) {
    multicastTTL = atoi(argv[4]);       // Command-line TTL
  }

  try {
    UDPSocket sock;

    sock.setMulticastTTL(multicastTTL);

    // Repeatedly send the string to the server
    for (;;) {
      sock.sendTo(sendString, strlen(sendString), servAddress, port);
      sleep(3);
    }
  } catch (SocketException &e) {
    cerr << e.what() << endl;
    exit(1);
  }

  return 0;
}
