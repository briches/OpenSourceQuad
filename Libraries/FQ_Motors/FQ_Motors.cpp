////========================================================================
	/*//*//*   FreeQuad   *//*//*
	FQ_Motors.cpp

	Author		: Brandon Riches
	Date		: August 2013

	Library designed to interface with and control brushless motor escs
    -----------------------------------------------------------------------*/
#include "FQ_Motors.h"
#include <FQ_QuadGlobalDefined.h>



FQ_MotorControl :: FQ_MotorControl(int num)
{
	this->NUM_MOTORS 	= num;
	this->MOTORS_ARMED 	= false;
	this->ESC_READY		= false;
	this->passiveMIN	= 1200;
	this->passiveMAX	= 1500;
	this->agroSTEP		= 200;
	this->agroBOOL		= false;
};

void FQ_MotorControl :: calibrateESC(int numESC)
{
	motor1.attach(MOTOR1PIN);
	motor2.attach(MOTOR2PIN);
	motor3.attach(MOTOR3PIN);
	motor4.attach(MOTOR4PIN);

	this->MOTORS_ARMED = true;

	// Calibration procedure
	// TODO: calibrate ESC

	this->ESC_READY = true;
};

void FQ_MotorControl :: startMotors(int startPercentDC)
{
	if(this->ESC_READY)
	{
		int dutyCycleTarget = map(startPercentDC, 0, 100, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

		int DC;
		for (DC = MIN_PULSE_WIDTH; DC <= dutyCycleTarget; DC++)
		{
			motor1.writeMicroseconds(DC);
			motor2.writeMicroseconds(DC);
			motor2.writeMicroseconds(DC);
			motor2.writeMicroseconds(DC);

			delayMicroseconds(DC*3);
		}

		motorControl.m1_DC = DC;
		motorControl.m2_DC = DC;
		motorControl.m3_DC = DC;
		motorControl.m4_DC = DC;
	}
};

void FQ_MotorControl :: updateMotors(	double pitchPID,
										double rollPID,
										double yawPID,
										double elevPID)
{
	if (MOTORS_ARMED)
	{
		#ifdef _PLUSconfig

			// Control angle
			motorControl.m1_DC 	+= pitchPID;
			motorControl.m4_DC 	-= pitchPID;
			motorControl.m2_DC	+= rollPID;
			motorControl.m3_DC 	-= rollPID;

			// TODO: Yaw, elev PID.

			// Restrict duty cycle to max/min
			motorControl.m1_DC = constrain(	motorControl.m1_DC,
											passiveMIN - agroBOOL * agroSTEP,
											passiveMAX + agroBOOL * agroSTEP);
			motorControl.m2_DC = constrain(	motorControl.m2_DC,
											passiveMIN - agroBOOL * agroSTEP,
											passiveMAX + agroBOOL * agroSTEP);
			motorControl.m3_DC = constrain(	motorControl.m3_DC,
											passiveMIN - agroBOOL * agroSTEP,
											passiveMAX + agroBOOL * agroSTEP);
			motorControl.m4_DC = constrain(	motorControl.m4_DC,
											passiveMIN - agroBOOL * agroSTEP,
											passiveMAX + agroBOOL * agroSTEP);


		#endif

		#ifdef _Xconfig


		//TODO:
		// Write motor logic for X config

		#endif
	}


}

void FQ_MotorControl :: motorDISARM()
{
	motor1.writeMicroseconds(MIN_PULSE_WIDTH);
	motor2.writeMicroseconds(MIN_PULSE_WIDTH);
	motor3.writeMicroseconds(MIN_PULSE_WIDTH);
	motor4.writeMicroseconds(MIN_PULSE_WIDTH);

	motor1.detach();
	motor2.detach();
	motor3.detach();
	motor4.detach();

	MOTORS_ARMED = false;
}
