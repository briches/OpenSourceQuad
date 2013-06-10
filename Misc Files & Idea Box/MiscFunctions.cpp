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
**//*
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
        @OVERVIEW
    Every once in a while, there's a spot in the code which we never want the runtime to get to, but
	if it did, the runtime wouldn't crash. In this case, we'd want to have a note of this saved to some
	external file for review. This function would be hard coded into the program to flag these errors.

        @INPUT
	Takes either 0 or 3 parameters:
	FILE_ID: Default -1 (unspecified) Name or corresponding file ID where the error occured.
	LINE_NO: Default -1 (unspecified) Line which the error occured. Due to the tedious nature of hard coding this,
	         the value entered here will usually be an approximation.
	CODE:    Default -1 (unspecified) Error code. Probably not important for our purposes.

        @OUTPUT
    Function returns >void
    Function writes an error log to an external file named ERR_LOG.csv. If no such file exists, it is created. If the error
    logger was unable to open the csv file, WARNING.txt is created to flag this error error.
    Err messages take the form:
    UTC_DATE (dd.mm.yyyy), UTC_TIME, FILE_ID, LINE_NO, CODE\n

    	@NOTES
    time.h may cause a data race on cocurrent access. This does not apply to an Arduino (I think).

        @DEPENDENCIES
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
        // This may be a good spot for recusivity... This would ensure that the error codes logs are written.
        err.open ("WARNING.txt", ios::out);
        err << "The error recorder was unable to record an error. This is an error. Errorception.";
        err.close();
    }
    err.close(); // Free the resource.
}





/**
MOTOR_WRITER_MAX_9000
	This is the meat and potatoes for the correction logic.
**/
void Corrector ()
{
	double Buffer = 0; 
	
	double MOT1_MOD,
	       MOT2_MOD,
	       MOT3_MOD,
	       MOT4_MOD;
	
	// Uses public variables in Class Control
	if (Control.alpha >= 0 + Buffer)
	{
		//I COMPLETELY FORGOT THE ARRANGEMENT OF THE MOTORS.
		MOT1_MOD = aPID_out + Control.Quadcopter_MotorSpeed.motor1s;
	}
	else
	{
		
	}
	
	if (Control.beta >= 0 + Buffer)
	{
		
	}
	else
	{
		
	}
}





/**
DATA BANK
        @OVERVIEW
    I know that this is completely redundant, but I think it's advantageous for us to have
    a consolidated source for all variables we'll need for future programming. If nothing else
    this might serve as a quick reference for all of our variables - I'm trying to mirror this as
    close as possible to what's already in place.

    But wait, there's more! Implement now, and you'll get DATABANK's constructor absolutely
    free of charge!
    Initializes ALL variables! No more NULL pointers!
**/
/*
class DATABANK
{
    DATABANK();
    private:
        struct Offsets
        {
            double  ax,             // Offsets from initial position sensor data
                    ay,
                    az,
                    wx,
                    wy,
                    wz;
        } Offsets;

        MMA8453_n0m1 accel;     // Accel class
        OseppGyro   gyro;		// Gyro class
        Servo       motor1;		// Motor 1
        Servo       motor2;		// Motor 2
        Servo       motor3;		// Motor 3
        Servo       motor4;		// Motor 4

    public:
        struct MotorSpeeds
        {
            int     MOTOR_XPOS,
                    MOTOR_XNEG,
                    MOTOR_YPOS,
                    MOTOR_YNEG;

                    #define motor1s MOTOR_XPOS
                    #define motor2s MOTOR_XNEG
                    #define motor3s MOTOR_YPOS
                    #define motor4s MOTOR_YNEG
        } MotorSpeeds;

        struct Settings
        {
            int     d_ScaleRange,
                    g_ScaleRange,
                    DLPF;
            bool    HighDef;
            double  g_threshold,
                    d_threshold;
        } Settings;

        struct Data
        {
            double  ax,
                    ay,
                    az,
                    wx,
                    wy,
                    wz,
                    prev_data[42],  // Stores 6 previous data sets, 1 current
                    t_current,      // Time @ call to update();
                    t_previous;     // Time @ previous call to update();
            int     freq;           // Frequency of calls to update();
        } Data;

        double alpha,               // Angle between x and z
               beta,                // Angle between y and z
               heading;             // Time integration of wz

} QC_DATABANK;

DATABANK::DATABANK ()
{
        getSettings();                          // Fill settings struct
        SI_convert();                           // Convert to SI
        get_Initial_Offsets();                  // Gets the initial Offsets
        initSensor();                           // Initializes the two sensors
        initMotors();                           // Initializes the 4 motors
        update();                               // Updates the structure
        setMotorSpeed();	                    // Sets a motor to a new speed
}
*/
