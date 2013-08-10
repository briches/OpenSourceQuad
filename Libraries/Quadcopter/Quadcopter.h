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

#include <QuadGlobalDefined.h>
#include <SENSORLIB.h>
#include <OseppGyro.h>
#include <I2C.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>



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
