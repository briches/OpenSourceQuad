/*=====================================================================
	OSQ_Quadcopter library
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: August 2013
	License		: GNU Public License

	This library is designed to abstract away some of the craft management functions
	from the main file (OSQ_Main.ino)

	Copyright (C) 2013  Brandon Riches

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	-----------------------------------------------------------------------------*/
#ifndef OSQ_QUADCOPTER_H_INCLUDED
#define OSQ_QUADCOPTER_H_INCLUDED


#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <OSQ_QuadGlobalDefined.h>
#include <OSQ_Kinematics.h>
#include <OSQ_SENSORLIB.h>
#include <OSQ_Motors.h>

#include <I2C.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>

/*=========================================================================
    General IO pins
    -----------------------------------------------------------------------*/
#define GREEN_LED 	27
#define RED_LED 	25
#define YELLOW_LED 	23


/*=========================================================================
    Polling Rates.
    -----------------------------------------------------------------------*/

#define _100HzPoll 		(10000)				// us Period of 100 Hz poll
#define	_50HzPoll		(20000)				// us Period of 75 Hz poll
#define _20HzPoll 		(50000)				// us Period of 50 Hz poll
#define _10HzPoll		(100000)			// us Period of 10 Hz poll

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
					SENSORLIB_gyro gyro,
					struct kinematicData *kinematics);

void mainProcess(	double pitchPID_out,
					double rollPID_out,
					class SENSORLIB_accel *accel,
					class SENSORLIB_mag	*mag,
					class SENSORLIB_gyro	*gyro,
					struct kinematicData *kinematics,
					struct fourthOrderData *fourthOrderXAXIS,
					struct fourthOrderData *fourthOrderYAXIS,
					struct fourthOrderData *fourthOrderZAXIS,
					struct OSQ_MotorControl *MotorControl);


void ERROR_LED(		int LED_SEL);

void getInitialOffsets(struct kinematicData *kinematics,
						SENSORLIB_accel accel,
						SENSORLIB_mag mag,
						SENSORLIB_gyro gyro);





#endif // OSQ_QUADCOPTER_H_INCLUDED
