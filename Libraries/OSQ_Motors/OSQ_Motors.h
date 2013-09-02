/*=====================================================================
	OSQ_Motors library
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: August 2013
	License		: GNU Public License

	This library is designed to interface with and control brushless motor escs

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

#ifndef OSQ_MOTORS_H_INCLUDED
#define OSQ_MOTORS_H_INCLUDED

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Servo.h>

/*=========================================================================
    Motors definitions
    -----------------------------------------------------------------------*/
#define MIN_PULSE_WIDTH       544     		// the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH      2400     		// the longest pulse sent to a servo

#define MOTOR1PIN			2
#define MOTOR2PIN			3
#define MOTOR3PIN			4
#define MOTOR4PIN			5

#define _PLUSconfig			1

//#define _Xconfig			1


//***********************************************/
/* Plus config
**************************************************
					  1
					  ||
				 	  ||
				 	  ||
				 	  ||
				  /  front  \
				  |  usb ^  |
		  2 ======|         |====== 3
				  |         |
				  \_________/
					  ||
					  ||
					  ||
					  ||
					  4
**************************************************
*/
//***********************************************/
struct motorSpeeds_t
{
	double m1_DC;
	double m2_DC;
	double m3_DC;
	double m4_DC;
};


class OSQ_MotorControl
{
	public:
		OSQ_MotorControl(int num = 4);
		Servo           	motor1;		// Motor 1
		Servo            	motor2;		// Motor 2
		Servo            	motor3;		// Motor 3
		Servo             	motor4;		// Motor 4

		motorSpeeds_t		motorSpeeds;

		// Function ideas //
		void calibrateESC(int numESC = 4);
		void motorDISARM();
		void startMotors();
		void updateMotors(double pitchPID,
						  double rollPID,
						  double yawPID = 0.F,
						  double elevPID = 0.F);



	private:
		bool 	MOTORS_ARMED;
		bool	ESC_READY;
		int 	NUM_MOTORS;

		int 	passiveMIN;
		int		passiveMAX;
		int		agroSTEP;

		bool 	agroBOOL;

};





#endif // FQ_MOTORS_H_INCLUDED
