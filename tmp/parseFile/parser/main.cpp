#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <stdlib.h>     /* for atoi() and exit() */

using namespace std;

class parseFile{
	public:
		parseFile(char* file);
		~parseFile();
		int Parse();
	private:
		ifstream dataFile;
		int processCount;
	  	// Location where the relevant data will be stored.
  		struct DataLayout
  		{
    			int    type;
    			string alarm;


    			string name;
    			string SHM;
    			string Node2RTAI;
    			string RTAI2Node;
    			string IP_RTAI;
    			string PORT_RTAI;

  		};
  		DataLayout theData[5];
};

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

int parseFile::Parse()
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
                processCount++;
                char buf2[ 80 ] = {0};
                string firstField2;
                string secondField2;
                string data2;




                while(firstField2 != ")")
                {
                    dataFile.getline( buf2, sizeof( buf2 ) );
                    istringstream istr2( string(buf2), ios_base::out );
                    istr2 >> firstField2 >> secondField2 >> data2;

                    if ( firstField2 == "name" )
                    {
                        theData[processCount].name = data2;
                    }
                    else if ( firstField2 == "SHM" )
                    {
                        theData[processCount].SHM = data2;
                    }
                    else if ( firstField2 == "Node2RTAI" )
                    {
                        theData[processCount].Node2RTAI = data2;
                    }
                    else if ( firstField2 == "Node2RTAI" )
                    {
                        theData[processCount].Node2RTAI = data2;
                    }
                    else if ( firstField2 == "RTAI2Node" )
                    {
                        theData[processCount].RTAI2Node = data2;
                    }
                    else if ( firstField2 == "IP_RTAI" )
                    {
                        theData[processCount].IP_RTAI = data2;
                    }
                    else if ( firstField2 == "PORT_RTAI" )
                    {
                        theData[processCount].PORT_RTAI = data2;
                    }
                }
            }


  	}

    cout << "Number of process = " << processCount + 1 << endl;

    for(int  i = 0; i< processCount + 1 ; i++)
    {
        cout << "theData["<< i << "].name  = " << theData[i].name
            << endl
            << "theData["<< i << "].SHM  = " << theData[i].SHM
            << endl
            << "theData["<< i << "].Node2RTAI  = " << theData[i].Node2RTAI
            << endl
            << "theData["<< i << "].RTAI2Node  = " << theData[i].RTAI2Node
            << endl
            << "theData["<< i << "].IP_RTAI  = " << theData[i].IP_RTAI
            << endl
            << "theData["<< i << "].PORT_RTAI  = " << theData[i].PORT_RTAI
            << endl;

        cout << "------------------------------------" << endl;
    }

}

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
