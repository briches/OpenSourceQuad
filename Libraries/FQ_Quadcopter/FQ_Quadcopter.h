/******************************************************/
/*//*//*   FreeQuad   *//*//*

Library designed to manage, update, and control the
state of quadcopter

Author  : Brandon Riches
Date     :	June 2013



******************************************************/
#ifndef CONTROL_H_INCLUDED
#define CONTROL_H_INCLUDED


#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <FQ_QuadGlobalDefined.h>
#include <FQ_Kinematics.h>
#include <FQ_SENSORLIB.h>
#include <FQ_OseppGyro.h>
#include <I2C.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>


/*===============================================
Sensor data, current and buffer.
-----------------------------------------------------------------------*/
double elev;
double vbatt;

/*===============================================
Time keeping for polling and interrupts
-----------------------------------------------------------------------*/
static double tpoll1;
static double tpoll2;
static double tpoll3;
static double tpoll4;


/***************************************************************************
 *! @FUNCTIONS
 ***************************************************************************/
bool initSensor(	SENSORLIB_accel accel,
					SENSORLIB_mag mag,
					OseppGyro gyro,
					struct kinematicData kinematics);

bool initMotors(	int speed);

void mainProcess(	double aPID_out,
					double bPID_out,
					SENSORLIB_accel accel,
					SENSORLIB_mag	mag,
					OseppGyro	gyro,
					struct kinematicData kinematics);

bool updateMotors(	double aPID_out,
					double bPID_out);

void ERROR_LED(		int LED_SEL);

void getInitialOffsets(struct kinematicData kinematics,
						SENSORLIB_accel accel,
						SENSORLIB_mag mag,
						OseppGyro gyro);





#endif // CONTROL_H_INCLUDED
