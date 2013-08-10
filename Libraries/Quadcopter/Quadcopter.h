/******************************************************
Library designed to manage, my_update, and Control the
state of quadcopter

Author  : Brandon Riches
Date     :	June 2013

my_updated for compatability with main polling loop and GPS interrupts


******************************************************/
#ifndef CONTROL_H_INCLUDED
#define CONTROL_H_INCLUDED


#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <SENSORLIB.h>
#include <OseppGyro.h>
#include <I2C.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>


#define Pi  			(3.14159F)			// Its pi.

#define _100HzPoll 		(10000)				// us Period of 100 Hz poll
#define	_75HzPoll		(13333)				// us Period of 75 Hz poll
#define _20HzPoll 		(50000)				// us Period of 50 Hz poll
#define _10HzPoll		(100000)			// us Period of 10 Hz poll

#define USRF_pin (0x0)						// TODO: Connect the USRF AN pin to this pin

#define MIN_PULSE_WIDTH       544     		// the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH      2400     		// the longest pulse sent to a servo


#define Gyro_Address    (0x69)				// I2C Address of the gyro

/*=========================================================================
    STATUS LEDs
    -----------------------------------------------------------------------*/
#define GREEN_LED 	27						// Digital I/O pin
#define RED_LED 	25						// Digital I/O pin
#define YELLOW_LED 	23						// Digital I/O pin

/*=========================================================================
    Battery Monitor
    -----------------------------------------------------------------------*/
#define NOMINAL_V	11.1


/*=========================================================================
    Chebyshev 4th order LPF
    -----------------------------------------------------------------------*/

#define ORDER 4

// Comment out one of these defines to select the coefficent set to use.
// "BrandonCoeffs" uses n = 4, r = 10, Wst = 0.1
// AeroQuad filter specs unknown.

#define AeroQuad
// #define BrandonCoeffs

#ifdef AeroQuad
	#define ORDER 4
	#define _b0  0.001893594048567
	#define _b1 -0.002220262954039
	#define _b2  0.003389066536478
	#define _b3 -0.002220262954039
	#define _b4  0.001893594048567

	#define _a0  1
	#define _a1 -3.362256889209355
	#define _a2  4.282608240117919
	#define _a3 -2.444765517272841
	#define _a4  0.527149895089809
#endif

#ifdef BrandonCoeffs
	#define ORDER 4
	#define _b0  0.267411759560506
	#define _b1 -1.018535973364803
	#define _b2  1.503499251793105
	#define _b3 -1.018535973364804
	#define _b4  0.267411759560506

	#define _a0   1.000000000000000
	#define _a1 -3.561271663800053
	#define _a2  4.788976593687705
	#define _a3 -2.881867760094174
	#define _a4  0.655413654391032
#endif

#define XAXIS   (0)
#define YAXIS	(1)
#define ZAXIS	(2)

struct fourthOrderData
{
  float  inputTm1,  inputTm2,  inputTm3,  inputTm4;
  float outputTm1, outputTm2, outputTm3, outputTm4;
};

/*===============================================
	Device settings (Options for sensors)
	- Explaining these is kinda hard, go read the data sheets.
	-----------------------------------------------------------------------*/
const int d_ScaleRange = FULL_SCALE_RANGE_250; // x250,x500,x1000,x2000
const int DLPF = 6;                 // 0,1,2,3,4,5,6,7 // See data sheet
const bool HighDef = true;          // Is accel output 2byte or 1byte

// These are the drift rates in d/(s*mus)
#define GYRO_DRIFT_RATE_X   -9E-10
#define GYRO_DRIFT_RATE_Y    3E-10
#define GYRO_DRIFT_RATE_Z   -3E-11


/***************************************************************************
 *! @QCopterData
 ***************************************************************************/
/*==========================================================================
	Main struct, holds settings, vars, and some data
    ------------------------------------------------------------------------------------------------------------*/
class Quadcopter
{
	public:
		/*===============================================
		Current motor speeds
		// Given in percent.
		-----------------------------------------------------------------------*/
		int motor1s;
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
		double mx[2];
		double my[2];
		double mz[2];
		double elev;
		double vbatt;

		double alpha_gyro;
		double beta_gyro;
		double heading_gyro;

		/*===============================================
		Time keeping for polling and interrupts
		-----------------------------------------------------------------------*/
		double tpoll1;
		double tpoll2;
		double tpoll3;
		double tpoll4;
		int freq;				    		// Frequency of calls to my_update();


		/*===============================================
		Sensor initial offset data
		-----------------------------------------------------------------------*/
		double io_ax;                 // Offsets from initial position sensor data
		double io_ay;
		double io_az;
		double io_wx;
		double io_wy;
		double io_wz;

		/*=========================================================================
		State variables
		- The program didnt like having these in the class.
		-----------------------------------------------------------------------*/
		double alpha;
		double beta;
		double heading;

		/***************************************************************************
		 *! @FUNCTIONS
		 ***************************************************************************/
		bool initSensor();                      							// Initializes the two sensors
		bool initMotors(int speed);                     							// Initializes the 4 motors
		void update(double aPID_out, double bPID_out);
		bool updateMotors(double aPID_out, double bPID_out);

		void ERROR_LED(int LED_SEL);


		/*===============================================
			Class initializations
			-----------------------------------------------------------------------*/
		SENSORLIB_accel    accel;       		// Accel class
		SENSORLIB_mag		 mag;				// Mag class
		OseppGyro       	gyro;				// Gyro class
		Servo             motor1;				// Motor 1
		Servo             motor2;				// Motor 2
		Servo             motor3;				// Motor 3
		Servo             motor4;				// Motor 4

		fourthOrderData   	fourthOrderXAXIS,
							fourthOrderYAXIS,
							fourthOrderZAXIS;

		private:
		float computeFourthOrder(float currentInput, struct fourthOrderData *filterParameters);
		void setupFourthOrder(void);
		void SI_convert();                 							// Convert to SI
		void get_Initial_Offsets();                           	// Gets the initial Offsets
};




#endif // CONTROL_H_INCLUDED
