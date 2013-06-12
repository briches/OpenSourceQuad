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
CORRECTION ALGORITHM CODE
**/
// Pseudocode Follows:
void Correction ()
{

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
