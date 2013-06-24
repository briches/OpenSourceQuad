/******************************************************
Library designed to manage, update, and Control the
state of quadcopter

Author  : Brandon Riches
Date     :	June 2013

Updated for compatability with main polling loop and GPS interrupts


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


#define Pi  (3.14159F)									// Its pi.
#define poll2_interrupt (50000)				// The latency period between readings of the USRF, in microseconds
#define poll3_interrupt (100000)				// The latency period between readings of the USRF, in microseconds
#define USRF_pin (0x0)								// Connect the USRF AN pin to this pin

/*=========================================================================
    I2C Addresses
    -----------------------------------------------------------------------*/
#define Accel_Address   (0x1D)
#define Gyro_Address    (0x69)

	/*===============================================
    Class initializations
    -----------------------------------------------------------------------*/
  MMA8453_n0m1  accel;       		// Accel class
  OseppGyro             gyro;			// Gyro class
  Servo                  motor1;			// Motor 1
  Servo                  motor2;			// Motor 2
  Servo                  motor3;			// Motor 3
  Servo                  motor4;			// Motor 4

	/*===============================================
    Device settings (Options for sensors)
    -----------------------------------------------------------------------*/
	const int d_ScaleRange = FULL_SCALE_RANGE_250; // x250,x500,x1000,x2000
	const int g_ScaleRange = FULL_SCALE_RANGE_2g;  // x2g,x4g,x8g
	const int DLPF = 7;                 // 0,1,2,3,4,5,6,7 // See data sheet
	const bool HighDef = true;          // Is accel output 2byte or 1byte
    const double g_threshold = 0.05; //Upper threshold for set zero from accel data
	const double d_threshold = 1;    //Upper threshold for set zero from gyro data


/***************************************************************************
 *! @QCopterData
 ***************************************************************************/
/*==========================================================================
	Main struct, holds settings, vars, and some data
    ------------------------------------------------------------------------------------------------------------*/
typedef struct QuadcopterVars
{
	/*===============================================
    Current motor speeds
    -----------------------------------------------------------------------*/
	int motor1s;                 // Motor speed for all 4 motors
	int motor2s;
	int motor3s;
	int motor4s;

	/*===============================================
    Sensor data, current and buffer.
    -----------------------------------------------------------------------*/
	double ax;                   // Basic sensor data
    double ay;
    double az;
    double wx;
    double wy;
    double wz;
    double elev;
    double prev_data[49];       // Stores 6 previous data sets, 1 current.
												// Prev_data is used for a moving average filter.

	/*===============================================
    Time keeping for polling and interrupts
    -----------------------------------------------------------------------*/
    double tpoll1;
    double tpoll2;
    double tpoll3;
    int freq;				    		// Frequency of calls to update();


	/*===============================================
    Sensor initial offset data
    -----------------------------------------------------------------------*/
    double io_ax;                 // Offsets from initial position sensor data
    double io_ay;
    double io_az;
    double io_wx;
    double io_wy;
    double io_wz;


	/*===============================================
    State variables
    -----------------------------------------------------------------------*/
	double alpha;                           								// Angle between x and z
	double beta;                            								// Angle between y and z
	double heading;                         							// Time integration of wz

};																			// Main struct name


/***************************************************************************
 *! @FUNCTIONS
 ***************************************************************************/
/*=========================================================================
    Function declarations
    -----------------------------------------------------------------------*/

void mov_avg();														// Runs a moving average

bool initSensor();                      							// Initializes the two sensors

bool initMotors();                     							// Initializes the 4 motors

void update();                         							 	// Updates the structure

void setMotorSpeed(int motor, int speed);		// Sets a motor to a new speed

void SI_convert(void);                 							// Convert to SI
void get_Initial_Offsets(void);         					// Gets the initial Offsets

QuadcopterVars           QV;         							// Creates the structure


#endif // CONTROL_H_INCLUDED
