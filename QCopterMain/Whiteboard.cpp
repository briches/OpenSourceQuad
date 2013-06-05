/*
 *  ----------
 *  Brandon Yue.
 *
 *  This space is being used as a sketchpad.
 *  CTRL + C is your friend.
 *  ----------

           ,-.
       ,--' ~.).
     ,'         `.
    ; (((__   __)))
    ;  ( (#) ( (#)
    |   \_/___\_/|
   ,"  ,-'    `__".
  (   ( ._   ____`.)--._        _
   `._ `-.`-' \(`-'  _  `-. _,-' `-/`.
    ,')   `.`._))  ,' `.   `.  ,','  ;
  .'  .     `--'  /     ).   `.      ;
 ;     `-        /     '  )         ;
 \                       ')       ,'
  \                     ,'       ;
   \               `~~~'       ,'
    `.                      _,'
      `.                ,--'
        `-._________,--'

 */

#include <iostream>
#include <cmath>

using namespace std;

/**
THINGS THAT MIGHT BE USEFUL
**/
enum cartesian {
	XPOS,
	XNEG,
	YPOS,
	YNEG,
	ZPOS,
	ZNEG
};

struct FrameData {
    int XPOS_MOTOR,     // In the actual implementation,
    int XNEG_MOTOR,     // replace these whith whatever
    int YPOS_MOTOR,     // system we have in place.
    int YNEG_MOTOR,
    float GYRO_X,
    float GYRO_Y,
    float GYRO_Z,
    float ACCEL_X,
    float ACCEL_Y,
    float ACCEL_Z,
    float TILT_DIR,	// Angle in degrees - Platform's Z+ tilt direction.
    float TILT_MAG	// Magnitude of tilt.
} data;


/**
CORRECTION ALGORITHM CODE
**/
/*  General Strategy:
        - Obtain current speeds assigned to each motor, accelerometer and gyro data. This is the current "frame"
        - Compute next speed to assign each motor based on this data.
          The exact algorithm we should use for this purpose has to be discussed.
          Things affecting our result:
            What is the current orientation of the platform?
            How fast are we currently rotating, and in what direciton?
            Have we already made appropreate corrections?
            Are we about to crash into terrain?
        - Send data to the motors, and store any data we need for the next frame.
    Eventually, the code should account for user input for movement.
 */
// Pseudocode Follows:
void Correction (FrameData &data)
{
	// We have the frame data accessable from data.myvar
	// The first thing we may want to consider is our current acceleration in the Z axis. After all, if
	// we're at - 9.81 in only Z, we're level and not tilting in any other direction.
	//
	// We need to take into account all of our parameters. Which ones are most important?
	
	float AccelBuffer = 0 //Number corresponding to our calculated noise in the accelerometer.
	
	if (data.ACCEL_Z + 9.81 < AccelBuffer and data.ACCEL_X < AccelBuffer and data.ACCEL_Y < AccelBuffer)
	{
		// This is our ideal scenario.
		// MyImmaginaryFunction(don't_change_anyhing);
	}
	else
	{
		MyFunction2 () 
		{
			/* I have no idea what the mathematics is for this, but here are some thoughts.
			
			If our tilt direction is any multiple of 1/2 rad, one motor must increase and one must decrease.
			The other two remain constant, since the tilt is along one of the platform's axises.
			These this is our edge case.
			*/
			if (not data.TILT_DIR % 90 < AccelBuffer) { // If the tilt direction is within error for one of the axises:
				// Get floor(90) for syntax clarification.
				r = data.TILT_DIR;
				myCount = 0
				while (r > -45)
				{
					r -= 90;
					myCount ++;
				}
				/*
				1  = Tilted towards X+ Y+
				2  = Tilted towards X- Y+
				3  = Tilted towards X- Y-
				4  = Tilted towards X+ Y-
				5+ = Datum data.TILT_DIR was not in the range 0 - 360. THIS SHOULD NOT HAPPEN.
				*/
				marker:
				switch myCount:
					case 1:
						INCREASE_XPOS();
						DECREASE_XNEG();
						break;
					case 2:
						INCREASE_YPOS();
						DECREASE_YNEG();
						break;
					case 3:
						INCREASE_XNEG();
						DECREASE_XPOS();
						break;
					case 4:
						INCREASE_YNEG();
						DECREASE_XPOS();
						break;
					default:
						flag_silent_error();
						myCount -= 4;
						goto marker;
			}
			else
			{
				/*
				Otherwise the tilt direction falls into one of four "quadrants" and all motors must make a change
				in speed. If this problem can be solved by addressing the X and Y components of the tilt
				independently then this should be a nonissue.
				
				We simply need a rough approximation of the relative force we get out of each incrimental change
				in motor speed.
				*/
			}
		}
		/*
		At this point, we should have calculated our desired speed for each motor. Now we simply
		save this data for the next iteration.
		*/
		XPOS_MOTOR = 0;
		XNEG_MOTOR = 0;
		YPOS_MOTOR = 0;
		YNEG_MOTOR = 0;
	}
}






/**
THIS IS JUST SO SOMETHING HAPPPENS WHEN THIS COMPILES & RUNS
**/
int main () {
    cout << "Hello world!" << endl;
    return 0;
}
