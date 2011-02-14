#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <stdio.h>

#include <stdlib.h>     /* for atoi() and exit() */


using namespace std;

struct DataLayout
  		{
    			string name;
    			string SHM;
    			string Node2RTAI;
    			string RTAI2Node;
    			string IP_RTAI;
    			string PORT_RTAI;
    			bool active;
    			int csock;
  		};

class parseFile{
	public:
		parseFile(char* file);
		~parseFile();
		int Parse(DataLayout * process);

		int getNumProcess();

	private:
		ifstream dataFile;
		int processCount;
	  	// Location where the relevant data will be stored.

  		DataLayout theData[5];
};
