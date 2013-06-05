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
enum Cartesian {
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
} data;


/**
CORRECTION ALGORITHM CODE
**/
/*  General Strategy:
        - Obtain current speeds assigned to each motor, accelerometer and gyro data. This is the current "frame"
        - Compute next speed to assign each motor based on this data.
          The exact algorithm we should use for this purpose has to be discussed.
          Things affecting our result:
            What is the current orientation of the platform? (Gyro)
            How fast are we currently rotating, and in what direciton? (Accelerometer)
            Have we already made appropreate corrections? (Current Speed Data)
            Are we about to crash into terrain? (IR/Sound Sensor)
        - Send data to the motors, and store any data we need for the next frame.
 */

// I DELETED ALL OF IT IN A SLEEPLESS RAGE.

/**
THIS IS JUST SO SOMETHING HAPPPENS WHEN THIS COMPILES & RUNS
**/
int main () {
    cout << "Hello world!" << endl;
    return 0;
}
