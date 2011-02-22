#include <stdio.h>
#include <string.h>
#include "parser.h"

parseFile::parseFile(char* file)
{
    processCount = -1;
    dataFile.open(file);



    // Verify the data file was opened; exit if not.
    if ( !dataFile )
    {
        cout << "Error:  Cannot open file " << file << endl;
        exit( 1 );
    }
}

parseFile::~parseFile()
{

}

int parseFile::Parse(DataLayout * process)
{

    // Parse the data file
    while ( ! dataFile.eof() )
    {

        char buf[ 80 ] = {0};
        string firstField;
        string secondField;
        string data;

        dataFile.getline( buf, sizeof( buf ) );

        istringstream istr( string(buf), ios_base::out );

        istr >> firstField >> secondField >> data;

        if ( firstField == "process(")
        {

            cout << firstField << endl;

            processCount++;
            char buf2[ 80 ] = {0};
            string firstField2;
            string secondField2;
            string data2;
            theData[processCount].active = 0;
            theData[processCount].csock = 0;

            while(firstField2 != ")")
            {

                dataFile.getline( buf2, sizeof( buf2 ) );

                istringstream istr2( string(buf2), ios_base::out );
                istr2 >> firstField2 >> secondField2 >> data2;

                process->active = 0;
                process->csock = 0;

                if(firstField2 != ")")
                {
                    cout << "  " << firstField2 << secondField2 << data2 << endl;

                    if ( firstField2 == "name" )
                    {
                        process[processCount].name = data2;
                    }else if ( firstField2 == "SHM_IN" )
                    {
                        process[processCount].SHM_IN = data2;
                    }else if ( firstField2 == "SHM_OUT" )
                    {
                        process[processCount].SHM_OUT = data2;
                    }
                    else if ( firstField2 == "Node2RTAI" )
                    {
                        process[processCount].Node2RTAI = data2;
                    }
                    else if ( firstField2 == "RTAI2Node" )
                    {
                        process[processCount].RTAI2Node = data2;
                    }
                    else if ( firstField2 == "IP_RTAI" )
                    {
                        process[processCount].IP_RTAI = data2;
                    }
                    else if ( firstField2 == "PORT_RTAI" )
                    {
                        process[processCount].PORT_RTAI = data2;
                    }
                    else if ( firstField2 == "Subscriber" )
                    {
                        process[processCount].Subscriber = data2;
                    }
                    else if ( firstField2 == "Publisher" )
                    {
                        process[processCount].Publisher = data2;
                    }

                }
            }
            cout << ")" << endl;
        }
    }

    for(int  i = 0; i< processCount + 1 ; i++)
    {

        cout << "Process ( " << endl;

        cout << "   process["<< i << "].name  = " << process[i].name
             << endl
             << "    process["<< i << "].SHM_IN  = " << process[i].SHM_IN
             << endl
             << "    process["<< i << "].SHM_OUT  = " << process[i].SHM_OUT
             << endl
             << "    process["<< i << "].Node2RTAI  = " << process[i].Node2RTAI
             << endl
             << "    process["<< i << "].RTAI2Node  = " << process[i].RTAI2Node
             << endl
             << "    process["<< i << "].IP_RTAI  = " << process[i].IP_RTAI
             << endl
             << "    process["<< i << "].PORT_RTAI  = " << process[i].PORT_RTAI
             << endl
             << "    process["<< i << "].active  = " << process[i].active
             << endl
             << "    process["<< i << "].csock  = " << process[i].csock
             << endl
             << "    process["<< i << "].Subscriber  = " << process[i].Subscriber
             << endl
             << "    process["<< i << "].Publisher  = " << process[i].Publisher
             << endl;
        cout << ") \n" << endl;

    }
}

int parseFile::getNumProcess()
{
    return processCount + 1;
}
