/******************************************************
Library designed to manage, update, and Control the
state of quadcopter

Author  : Brandon Riches
Date    : May 2013


    *Intructions to use*
    1) Call initSensor() in setup().
    2) Call get_Initial_Offsets in setup().
    3) At least, call update() in loop().


******************************************************/
#ifndef CONTROL_H_INCLUDED
#define CONTROL_H_INCLUDED


#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <MMA8453_n0m1.h>
#include <OseppGyro.h>
#include <I2C.h>
#include <math.h>
#include <Servo.h>


const double Pi = 3.14159;

/*
// Since I don't have a familiar IDE at the university I'm sticking this here.
// General Strategy:
// angle_correct()
// 	This function is the umbrella for the correction algorithm. Calls subfunctions and assigns "Locally Global" variables needed in this scope.
// evaluate_angle()
//	Creates a speed to return to the motors based on the current motor speed and gyro rotation.
//	Takes arguments:
//		float MyGyro - Array of the current rotation of the craft. Stored as X,Y,Z rotation.
//		int MySpeeds - Array of the current assigned speed of the motors.

// enumeration for cartesian coordinate sysem. May be usefull when constructing algorithms. Not sure.
// Creates a data type called Cartesian with 6 possible values.
enum Cartesian {
	XPOS,
	XNEG,
	YPOS,
	YNEG,
	ZPOS,
	ZNEG
}

// Pseudocode for correction algorithm since I don't know most of the relevant function calls.
// Assume global array MyArray which has values for:
//  - Last assigned speed for each motor
//  - Maybe other stuff

void angle_correct(){
	float CurrentRot[3] = {get_x_rot(), get_y_rot(), get_z_rot();}
	// Assume assigned speeds are arranged by:
	// XPOS, XNEG, YPOS, YNEG
	int CurrentSpd[4] = {MyArray[0], MyArray[1], MyArray[2], MyArray[3]};
	evaluate_angle(CurrentRot[], CurrentSpd[])
}

int evaluate_angle(){//ran out of time.};



*/



/*=========================================================================
    I2C Addresses
    -----------------------------------------------------------------------*/
#define Accel_Address   (0x1D)
#define Gyro_Address    (0x69)

 /*=========================================================================
    Device settings (Options for sensors)
    -----------------------------------------------------------------------*/
const int d_ScaleRange = FULL_SCALE_RANGE_250; // x250,x500,x1000,x2000
const int g_ScaleRange = FULL_SCALE_RANGE_2g;  // x2g,x4g,x8g
const int DLPF = 7;                 // 0,1,2,3,4,5,6,7 // See data sheet
const bool HighDef = true;          // Is accel output 2byte or 1byte

const double g_threshold = 0.05; //Upper threshold for set zero from accel data
const double d_threshold = 1;    //Upper threshold for set zero from gyro data


/*=========================================================================
	Calculated state variables from sensor data and motor speeds
    -----------------------------------------------------------------------*/

typedef struct Quadcopter_MotorSpeed_s
{
	int motor1s;                 // Motor speed for all 4 motors
	int motor2s;
	int motor3s;
	int motor4s;
} Quadcopter_MotorSpeed;



/*=========================================================================
    Sensor data and timestamps
    -----------------------------------------------------------------------*/
typedef struct Quadcopter_Data_s
{

    double ax;                   // Basic sensor data
    double ay;
    double az;
    double wx;
    double wy;
    double wz;
    double prev_data[42];       // Stores 6 previous data sets, 1 current
    double t_current;           // Time @ call to update();
    double t_previous;          // Time @ previous call to update();
    int freq;				    // Frequency of calls to update();


} Quadcopter_Data;

/*=========================================================================
    DEVICE SETTINGS STRUCTURE
    -----------------------------------------------------------------------*/
typedef struct Device_Settings_s
{
    int d_ScaleRange;
    int g_ScaleRange;
    int DLPF;
    bool HighDef;
    double g_threshold;
    double d_threshold;

} Device_Settings;

/*=========================================================================
    DEVICE SETTINGS STRUCTURE
    -----------------------------------------------------------------------*/
typedef struct Initial_Offsets_s
{
    double ax;                   // Offsets from initial position sensor data
    double ay;
    double az;
    double wx;
    double wy;
    double wz;

} Initial_Offsets;


/*=========================================================================
    CLASS DECLARATION
    -----------------------------------------------------------------------*/
class Control
{
    public:

        bool initSensor();                      // Initializes the two sensors
        bool initMotors();                      // Initializes the 4 motors
        void update();                          // Updates the structure
        void setMotorSpeed(int motor, int speed);		// Sets a motor to a new speed

        Quadcopter_Data           Data;         // Creates the structure
        Device_Settings         Settings;
        Quadcopter_MotorSpeed MotorSpeeds;

        double alpha;                           // Angle between x and z
		double beta;                            // Angle between y and z
		double heading;                         // Time integration of wz



    private:

        Initial_Offsets         Offsets;        // Creates the structure

        MMA8453_n0m1              accel;        // Accel class
        OseppGyro                  gyro;		// Gyro class
        Servo                    motor1;		//  Motor 1
        Servo                    motor2;		// Motor 2
        Servo                    motor3;		// Motor 3
        Servo                    motor4;		// Motor 4

        void getSettings(void);                 // Fill settings struct
        void SI_convert(void);                  // Convert to SI
        void get_Initial_Offsets(void);         // Gets the initial Offsets
};
#endif // CONTROL_H_INCLUDED
