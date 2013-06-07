/**
  *  Brandon Yue
  *
  *  This file will hold functions that I think might be useful somewhere.
  *
  */

#include <iostream>
#include <fstream>
#include <time.h>
using namespace std;

/**
FUNCTIONS:
**/
void SILENT_ERR(int, int, int);
void SILENT_ERR();


/**
MAIN: FOR DEBUG PURPOSES. COMMENT THIS OUT WHEN USING THIS FILE.
**/
int main ()
{
    cout << "Hello World!" << endl;
    SILENT_ERR(1,2,3);
    SILENT_ERR();
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
	Takes either 0 or 3 parameters:
	FILE_ID: Default -1 (unspecified) Name or corresponding file ID where the error occured.
	LINE_NO: Default -1 (unspecified) Line which the error occured. Due to the tedious nature of hard coding this,
	         the value entered here will usually be an approximation.
	CODE:    Default -1 (unspecified) Error code. Probably not important for our purposes.

        OUTPUT
    Function returns @void
    Function writes an error log to an external file named ERR_LOG.csv. If no such file exists, it is created. If the error
    logger was unable to open the csv file, WARNING.txt is created to flag this error error.
    Err messages take the form:
    UTC_DATE (dd.mm.yyyy), UTC_TIME, FILE_ID, LINE_NO, CODE\n

    	NOTES
    time.h may cause a data race on cocurrent access. This does not apply to an Arduino (I think).

        Dependent on:
    #include <iostream>
    #include <fstream>
    #include <time.h>

 **/

void SILENT_ERR (int FILE_ID = -1, int LINE_NO = -1, int CODE = -1)
{
    char Date [80];
	time_t rawtime;
	struct tm * UTC_TIME;
	time (&rawtime);
	UTC_TIME = gmtime (&rawtime);
	strftime(Date, 80, "%d.%m.%Y", UTC_TIME);

    ofstream err;
    err.open ("ERR_LOG.csv", ios::out | ios::app); // open the resource: ERR_LOG.csv in output & append mode

    if (err.is_open()) // Exception CYA.
    {
    	err << Date << ","
    	    << UTC_TIME->tm_hour%24 << ":" << UTC_TIME->tm_min << ","
    	    << FILE_ID  << ","
    		<< LINE_NO  << ","
    		<< CODE     << '\n';
    }
    else
    {
        err.open ("WARNING.txt", ios::out);
        err << "The error recorder was unable to record an error. This is an error. Errorception.";
        err.close();
    }
    err.close(); // Free the resource.
}
/**NO ARGS OVERLOAD**/
void SILENT_ERR ()
{
    char Date [80];
	time_t rawtime;
	struct tm * UTC_TIME;
	time (&rawtime);
	UTC_TIME = gmtime (&rawtime);
	strftime(Date, 80, "%d.%m.%Y", UTC_TIME);

    ofstream err;
    err.open ("ERR_LOG.csv", ios::out | ios::app); // open the resource: ERR_LOG.csv in output & append mode

    if (err.is_open()) // Exception CYA.
    {
    	err << Date << ","
    	    << UTC_TIME->tm_hour%24 << ":" << UTC_TIME->tm_min << ","
    	    << "-1" << ","
    		<< "-1" << ","
    		<< "-1" << '\n';
    }
    else
    {
        err.open ("WARNING.txt", ios::out);
        err << "The error recorder was unable to record an error. This is an error. Errorception.";
        err.close();
    }
    err.close(); // Free the resource.
}
