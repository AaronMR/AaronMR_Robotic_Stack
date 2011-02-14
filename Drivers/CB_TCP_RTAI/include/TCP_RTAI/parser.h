#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <stdio.h>

#include <stdlib.h>     /* for atoi() and exit() */


using namespace std;

struct DataLayout
{
    int putamierda;
    string name;
    string SHM;
    string Node2RTAI;
    string RTAI2Node;
    string IP_RTAI;
    string PORT_RTAI;
    bool active;
    int csock;
    string Subscriber;
    string Publisher;

    //mutex
    pthread_mutex_t mutex;

    bool canRecv;
    bool canSend;

};

class parseFile
{
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
