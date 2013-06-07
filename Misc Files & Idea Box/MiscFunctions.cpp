/**
  *  Brandon Yue
  *
  *  This file will hold functions that I think might be useful somewhere.
  *
  */

/**
FUNCTIONS:
**/
void SILENT_ERR(int, int, int);



/**
MAIN: FOR DEBUG PURPOSES. COMMENT THIS OUT WHEN USING THIS FILE.
**//*
int main ()
{
    cout << "Hello World!" << endl;
    SILENT_ERR(1,1,1);
    SILENT_ERR(2,2,2);
    return 0;
}
//*/




/**
SILENT ERROR FLAGGER.
    Every once in a while, there's a spot in the code which we never want the runtime to get to, but
	if it did, the runtime wouldn't crash. In this case, we'd want to have a note of this saved to some
	external file for review. This function would be hard coded into the program to flag these errors.

	It actually works.

        INPUT
	Takes optional parameters:
	FILE_ID: Default -1 (unspecified) Name or corresponding file ID where the error occured.
	LINE_NO: Default -1 (unspecified) Line which the error occured. Due to the tedious nature of hard coding this,
	         the value entered here will usually be an approximation.
	CODE:    Default -1 (unspecified) Error code. Probably not important for our purposes.

        OUTPUT
    Function returns @void
    Writes an error log to an external file named ERR_LOG.csv
    Err messages take the form:
    UTC_TIME, FILE_ID, LINE_NO, CODE\n

    	NOTES
    Causes runtime exception on cocurrent access, but since we're not using a distributed system
    this is a nonissue. If we upgrade to a 4X XEON platform, remind me to recode this.
**/

/**
 * Dependent on:
 **/
#include <iostream>
#include <fstream>
#include <time.h>
using namespace std;

//FUNCTION:
void SILENT_ERR (int FILE_ID = -1, int LINE_NO = -1, int CODE = -1)
{
	/// REMEMBER TO IMPLEMENT THIS CORRECTLY.
	time_t rawtime;
	struct tm * UTC_TIME;
	time (&rawtime);
	UTC_TIME = gmtime ( &rawtime );

    ofstream err;
    err.open ("ERR_LOG.csv", ios::out | ios::app); // open the resource: ERR_LOG.csv in output & append mode

    if (err.is_open()) // Exception CYA.
    {
    	err << UTC_TIME->tm_hour%24 << ":" << UTC_TIME->tm_min << ","
    	    << FILE_ID  << ","
    		<< LINE_NO  << ","
    		<< CODE     << '\n';
    }
    else
    {
    	//ERROR-CEPTION! We don't really need anything here, or it basically becomes a recursive function.
    }
    err.close(); // Free the resource.
}
