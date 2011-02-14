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
		void start();
		int open();
		int Parse();
	private:
		ifstream dataFile;
	  	// Location where the relevant data will be stored.
  		struct DataLayout
  		{
    			int    type;
    			string alarm;
  		};
  		DataLayout theData;
};

parseFile::parseFile(char* file)
{
	// Attempt to open the data file.
	ifstream dataFile( file );

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

void parseFile::start()
{

}

int parseFile::open()
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

    		if ( firstField == "Type" )
    		{
      			theData.type = atoi( data.c_str() );
    		}
    		else if ( firstField == "Alarm" )
    		{
      			theData.alarm = data;
    		}
  	}
	cout << "theData.type  = " << theData.type
       << endl
       << "theData.alarm = " << theData.alarm
       << endl;
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
	

/*
  // Attempt to open the data file.
  ifstream dataFile( argv[1] );

  // Verify the data file was opened; exit if not.
  if ( !dataFile )
  {
    cout << "Error:  Cannot open file " << argv[1] << endl;
    exit( 1 );
  }
*/

/*
  // Location where the relevant data will be stored.
  struct DataLayout
  {
    int    type;
    string alarm;
  };
  DataLayout theData;
*/

/*
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

    if ( firstField == "Type" )
    {
      theData.type = atoi( data.c_str() );
    }
    else if ( firstField == "Alarm" )
    {
      theData.alarm = data;
    }
  }
*/

/*
  cout << "theData.type  = " << theData.type
       << endl
       << "theData.alarm = " << theData.alarm
       << endl;
*/
}  
