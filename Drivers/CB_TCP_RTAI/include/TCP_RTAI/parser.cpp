#include <stdio.h>
#include <string.h>
#include "parser.h"

parseFile::parseFile(char* file)
{

    processCount = -1;
    // Attempt to open the data file.
    //ifstream dataFileTemp();
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
                    //theData[processCount].name = data2;

                    //cout << "en name" << endl;
                    //process->name = "Process_3";
                    process[processCount].name = data2;

                }
                else if ( firstField2 == "SHM" )
                {
                    //theData[processCount].SHM = data2;
                    //cout << "en SHM" << endl;
                    //process->SHM = "SHM_3";
                    process[processCount].SHM = data2;

                }
                else if ( firstField2 == "Node2RTAI" )
                {
                    //theData[processCount].Node2RTAI = data2;
                    //cout << "en Node2RTAI" << endl;
                    //process->Node2RTAI = "Joy";
                    process[processCount].Node2RTAI = data2;

                }
                else if ( firstField2 == "RTAI2Node" )
                {
                    //theData[processCount].RTAI2Node = data2;
                    //cout << "en RTAI2Node" << endl;
                    //process->RTAI2Node = "Joy";
                    process[processCount].RTAI2Node = data2;

                }
                else if ( firstField2 == "IP_RTAI" )
                {
                    //theData[processCount].IP_RTAI = data2;
                    //cout << "en IP_RTAI" << endl;
                    //process->IP_RTAI = "127.0.0.1";
                    //process->IP_RTAI = "140.78.133.43";

                    process[processCount].IP_RTAI = data2;

                }
                else if ( firstField2 == "PORT_RTAI" )
                {
                    //theData[processCount].PORT_RTAI = data2;
                    //cout << "en PORT_RTAI" << endl;
                    //process->PORT_RTAI = "1101";
                    process[processCount].PORT_RTAI = data2;

                }
                else if ( firstField2 == "Subscriber" )
                {
                    //theData[processCount].Subscriber = data2;
                    //cout << "en Subscriber" << endl;
                    //process->Subscriber = "joy";
                    process[processCount].Subscriber = data2;

                }
                else if ( firstField2 == "Publisher" )
                {
                    //theData[processCount].Publisher = data2;

                    //cout << "en Publisher" << endl;
                    //process->Publisher = "maki2";
                    process[processCount].Publisher = data2;


                }

                }
            }
            cout << ")" << endl;

        }


    }
    //memcpy(process, theData, sizeof(theData));

    for(int  i = 0; i< processCount + 1 ; i++)
    {

        //memcpy(process, theData, sizeof(theData));

        cout << "Process ( " << endl;

        cout << "   process["<< i << "].name  = " << process[i].name
             << endl
             << "    process["<< i << "].SHM  = " << process[i].SHM
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

    /*
    process->active = 0;
    process->csock = 0;
    process->IP_RTAI = "127.0.0.1";
    process->IP_RTAI = "140.78.133.43";
    process->name = "Process_3";
    process->Node2RTAI = "Joy";
    process->PORT_RTAI = "1101";
    process->Publisher = "maki2";
    process->RTAI2Node = "Joy";
    process->SHM = "SHM_3";
    process->Subscriber = "joy";
    */
}

int parseFile::getNumProcess()
{
    return processCount + 1;
}


/*
int main( int argc, char** argv )
{
  	// Ensure that the data file name is supplied on the command line.
  	// If not, then exit.
  	if ( argc != 2 )
  	{
    		cout << "Usage: " << argv[0] << " <data file>" << endl;
    		exit( 1 );
  	}

	parseFile parser(argv[1]);
	parser.Parse();

}
*/
