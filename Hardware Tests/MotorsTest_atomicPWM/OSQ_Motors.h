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

#define USE_300HZ_ESC

#ifdef USE_300HZ_ESC
	#define PWM_FREQUENCY   300
#endif

#ifdef USE_400HZ_ESC
	#define PWM_FREQUENCY   400
#endif

#define PRESCALER			8
#define PWM_COUNTER_PERIOD	(F_CPU/PRESCALER/PWM_FREQUENCY)


#define MIN_COMMAND      	1000     		// the shortest pulse
#define MAX_COMMAND     	2000     		// the longest pulse

#define MOTOR1PIN			2				// Front Motor PORTE PE4
#define MOTOR2PIN			3				// Left Motor  PORTE PE5
#define MOTOR3PIN			5				// Right Motor PORTE PE3
#define MOTOR4PIN			6				// Back	Motor  PORTH PH3

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
				  /      front        \
                                  |      usb ^      |
                2 ======|                      |====== 3
				  |                     |
				  \_________/
					    ||
					    ||
					    ||
					    ||
					    4
**************************************************
*/
//***********************************************/

int motorSpeeds[4] = {0, 0, 0, 0};

void initializePWM()
{
	// Using counters 3 and 4
	// These are 16bit counter registers
	DDRE = DDRE | B00111000;			// Set PE3-5, OC3A, OC3B, OC3C to outputs
	DDRH = DDRH | B00001000;			// Set PH3, OC4A to outputs

	 // Init PWM Timer 3                                       // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM
        TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
        TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
        ICR3 = PWM_COUNTER_PERIOD;                                // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.

	// Init PWM Timer 4
	TCCR4A = (1<<WGM41)|(1<<COM4A1);
	TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
	ICR4 = PWM_COUNTER_PERIOD;
};

void writeMotors()
{
	OCR3B = motorSpeeds[0] * 2;
	OCR3C = motorSpeeds[1] * 2;
	OCR3A = motorSpeeds[2] * 2;
	OCR4A = motorSpeeds[3] * 2;
};

void commandAllMotors(int speed)
{
	motorSpeeds[0] = speed;
	motorSpeeds[1] = speed;
	motorSpeeds[2] = speed;
	motorSpeeds[3] = speed;
};

class OSQ_MotorControl
{
	public:
		OSQ_MotorControl(int num = 4);

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
		int 	        NUM_MOTORS;

		int 	        passiveMIN;
		int		passiveMAX;
		int		agroSTEP;

		bool 	agroBOOL;

};

OSQ_MotorControl :: OSQ_MotorControl(int num)
{
	this->NUM_MOTORS 	= num;
	this->MOTORS_ARMED 	= false;
	this->ESC_READY		= false;
	this->passiveMIN	= 1300;
	this->passiveMAX	= 1700;
	this->agroSTEP		= 100;
	this->agroBOOL		= false;
};

void OSQ_MotorControl :: calibrateESC(int numESC)
{
	initializePWM();

	this->MOTORS_ARMED = true;

	// Calibration procedure
	// TODO: calibrate ESC

	this->ESC_READY = true;
};

void OSQ_MotorControl :: startMotors()
{
	if(this->ESC_READY)
	{
		int dutyCycleTarget = (MIN_COMMAND);

		int DC;
		for (DC = 0; DC <= dutyCycleTarget; DC++)
		{
			motorSpeeds[0] = DC;
			motorSpeeds[1] = DC;
			motorSpeeds[2] = DC;
			motorSpeeds[3] = DC;
			writeMotors();
			delayMicroseconds(DC*3);
		}

	}
};

void OSQ_MotorControl :: updateMotors(	double pitchPID,
										double rollPID,
										double yawPID,
										double elevPID)
{
	if (MOTORS_ARMED)
	{
		#ifdef _PLUSconfig

			// Control angle
			motorSpeeds[0] 	+= rollPID;
			motorSpeeds[1] 	-= rollPID;
			motorSpeeds[2]	+= pitchPID;
			motorSpeeds[3]	-= pitchPID;

			// TODO: Yaw, elev PID.

			// Restrict duty cycle to max/min
			motorSpeeds[0] = constrain(	motorSpeeds[0],
											passiveMIN - agroBOOL * agroSTEP,
											passiveMAX + agroBOOL * agroSTEP);
			motorSpeeds[1] = constrain(	motorSpeeds[1],
											passiveMIN - agroBOOL * agroSTEP,
											passiveMAX + agroBOOL * agroSTEP);
			motorSpeeds[2] = constrain(	motorSpeeds[2],
											passiveMIN - agroBOOL * agroSTEP,
											passiveMAX + agroBOOL * agroSTEP);
			motorSpeeds[3] = constrain(	motorSpeeds[3],
											passiveMIN - agroBOOL * agroSTEP,
											passiveMAX + agroBOOL * agroSTEP);


			writeMotors();	// In OSQ_atomicPWM
		#endif

		#ifdef _Xconfig


		//TODO:
		// Write motor logic for X config

		#endif
	}


};

void OSQ_MotorControl :: motorDISARM()
{
	motorSpeeds[0] = 1000;
	motorSpeeds[1] = 1000;
	motorSpeeds[2] = 1000;
	motorSpeeds[3] = 1000;

	MOTORS_ARMED = false;
};



#endif // FQ_MOTORS_H_INCLUDED
