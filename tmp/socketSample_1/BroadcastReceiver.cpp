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

#include "PracticalSocket.h"   // For UDPSocket and SocketException
#include <iostream>            // For cout and cerr
#include <cstdlib>             // For atoi()

const int MAXRCVSTRING = 4096; // Longest string to receive

int main(int argc, char *argv[]) {

  if (argc != 2) {                  // Test for correct number of parameters
    cerr << "Usage: " << argv[0] << " <Local Port>" << endl;
    exit(1);
  }

  unsigned short echoServPort = atoi(argv[1]);     // First arg:  local port

  try {
    UDPSocket sock(echoServPort);                
  
    char recvString[MAXRCVSTRING + 1]; // Buffer for echo string + \0
    string sourceAddress;              // Address of datagram source
    unsigned short sourcePort;         // Port of datagram source
    int bytesRcvd = sock.recvFrom(recvString, MAXRCVSTRING, sourceAddress, 
                                  sourcePort);
    recvString[bytesRcvd] = '\0';  // Terminate string
    
    cout << "Received " << recvString << " from " << sourceAddress << ": "
         << sourcePort << endl;
  } catch (SocketException &e) {
    cerr << e.what() << endl;
    exit(1);
  }

  return 0;
}
