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
 /**
 WHITEBOARD NOTES
 **/
 /*
    Nothing to see here. for now.
 */





#include <iostream>
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <time.h>

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
/*
struct New_Motor_Properties {

    cartesian ID;
    int
        speed,
        other_property_1,
        other_property_2;
    float
        float_thing_1,
        float_thing_2;

} motor_XPOS, motor_XNEG, motor_YPOS, motor_YNEG;
*/
struct FrameData {
    int     XPOS_MOTOR,     // In the actual implementation,
            XNEG_MOTOR,     // replace these whith whatever
            YPOS_MOTOR,     // system we have in place.
            YNEG_MOTOR;
    float   GYRO_X,
            GYRO_Y,
            GYRO_Z,
            ACCEL_X,
            ACCEL_Y,
            ACCEL_Z,
            TILT_DIR,	// Angle in degrees - Platform's Z+ tilt direction.
            TILT_MAG;	// Magnitude of tilt.
} data;





/**
PLACEHOLDER FUNCTIONS
**/
void PLACEHOLDER_CMS (int speed, cartesian ID){} //     Change Motor Speed
void PLACEHOLDER_FSE (int code){} //                    Flags silent errors





/**
AIR BUBBLE IN A GLASS TUBE 9001
    This function is meant to provide a convienient means of determining how much
    of a change in the assigned motor speed

    Notes:
        - Function tends towards OVER correction.
**/
int AIR_BUBBLE_IN_GLASS_TUBE_9001 (float angle, bool mode)
{
    const int   UNIT_ANGLE = 0; // For every UNIT_ANGLE degrees we are tilted
    const float UNIT_SPEED = 0; // modify the speed by UNIT_SPEED

    angle = abs(angle);
    int result = 0;

    while (angle > 0)
    {
        angle  -= UNIT_ANGLE;
        result += UNIT_SPEED;
    }

    if (mode)
    {
        return result;
    }
    else
    {
        return (-1 * result);
    }
}
/**DOUBLE OVERLOAD**/
int AIR_BUBBLE_IN_GLASS_TUBE_9001 (double angle, double mode)
{
    const int    UNIT_ANGLE = 0; // For every UNIT_ANGLE degrees we are tilted
    const double UNIT_SPEED = 0; // modify the speed by UNIT_SPEED

    angle = abs(angle);
    int result = 0;

    while (angle > 0)
    {
        angle -= UNIT_ANGLE;
        result += UNIT_SPEED;
    }

    if (mode)
    {
        return result;
    }
    else
    {
        return (-1 * result);
    }
}





/**
CORRECTION ALGORITHM CODE
**/
/**
    General Strategy:
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
 **/
// Pseudocode Follows:
void Correction (FrameData &data)
{
	// We have the frame data accessable from data.myvar
	// The first thing we may want to consider is our current acceleration in the Z axis. After all, if
	// we're at - 9.81 in only Z, we're level and not tilting in any other direction.
	//
	// We need to take into account all of our parameters. Which ones are most important?

	float AccelBuffer = 0; //Number corresponding to our calculated noise in the accelerometer.

	if (data.ACCEL_Z + 9.81 < AccelBuffer and data.ACCEL_X < AccelBuffer and data.ACCEL_Y < AccelBuffer)
	{
		// This is our ideal scenario.
		// MyImmaginaryFunction(don't_change_anyhing);
	}
	else
	{
		//MyFunction2 ()
		{
			/* I have no idea what the mathematics is for this, but here are some thoughts.

			If our tilt direction is any multiple of 1/2 rad, one motor must increase and one must decrease.
			The other two remain constant, since the tilt is along one of the platform's axises.
			These are our simple edge case.
			*/
			if (data.TILT_DIR % 90 < AccelBuffer) { // If the tilt direction is within error for one of the axises:
				// Get floor(90) for syntax clarification.
				float r = data.TILT_DIR;
				int myCount = 0;
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
				switch (myCount)
				{
				    /**
				    case N:
				    {
                        FUNCTION_TO_CHANGE_MOTOR_SPEED (CURRENT_SPEED + FUNCTION_TO_MAKE_ADJUSTMENT (CURRENT_TILT, INCREASE_SPEED), TARGET_MOTOR)
                        FUNCTION_TO_CHANGE_MOTOR_SPEED (CURRENT_SPEED + FUNCTION_TO_MAKE_ADJUSTMENT (CURRENT_TILT, DECREASE_SPEED), TARGET_MOTOR)
				    }
				    **/
					case 1:
					{
					    PLACEHOLDER_CMS (data.XPOS_MOTOR + AIR_BUBBLE_IN_GLASS_TUBE_9001 (data.TILT_MAG, true ), XPOS);
					    PLACEHOLDER_CMS (data.XNEG_MOTOR + AIR_BUBBLE_IN_GLASS_TUBE_9001 (data.TILT_MAG, false), XNEG);
					    break;
					}
					case 2:
					{
					    PLACEHOLDER_CMS (data.YPOS_MOTOR + AIR_BUBBLE_IN_GLASS_TUBE_9001 (data.TILT_MAG, true ), YPOS);
					    PLACEHOLDER_CMS (data.YNEG_MOTOR + AIR_BUBBLE_IN_GLASS_TUBE_9001 (data.TILT_MAG, false), YNEG);
					    break;
					}
					case 3:
					{
					    PLACEHOLDER_CMS (data.XPOS_MOTOR + AIR_BUBBLE_IN_GLASS_TUBE_9001 (data.TILT_MAG, false), XPOS);
					    PLACEHOLDER_CMS (data.XNEG_MOTOR + AIR_BUBBLE_IN_GLASS_TUBE_9001 (data.TILT_MAG, true ), XNEG);
					    break;
					}
					case 4:
					{
					    PLACEHOLDER_CMS (data.YPOS_MOTOR + AIR_BUBBLE_IN_GLASS_TUBE_9001 (data.TILT_MAG, false), YPOS);
					    PLACEHOLDER_CMS (data.YNEG_MOTOR + AIR_BUBBLE_IN_GLASS_TUBE_9001 (data.TILT_MAG, true ), YNEG);
					    break;
					}
					default:
                        /**
                        IF IMPLEMENTED CORRECTLY, THE PROGRAM NEVER EXECUTES THIS BLOCK
                        **/
						PLACEHOLDER_FSE (-1);
						myCount -= 4;
				}
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
	}
}






/**
THIS IS JUST SO SOMETHING HAPPPENS WHEN THIS COMPILES & RUNS
**/
int main () {
    time_t rawtime;
    struct tm * ptm;
    time ( &rawtime );
    ptm = gmtime ( &rawtime );
    cout << "Hello world!" << endl;
    printf ("GMT: %2d:%02d\n", (ptm->tm_hour)%24, ptm->tm_min);
    return 0;
}
