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

void FQ_MotorControl :: startMotors()
{
	if(this->ESC_READY)
	{
		int dutyCycleTarget = (passiveMAX + passiveMIN) / 2;

		int DC;
		for (DC = MIN_PULSE_WIDTH; DC <= dutyCycleTarget; DC++)
		{
			motor1.writeMicroseconds(DC);
			motor2.writeMicroseconds(DC);
			motor3.writeMicroseconds(DC);
			motor4.writeMicroseconds(DC);
			Serial.println(DC);
			delayMicroseconds(DC*3);
		}

		motorSpeeds.m1_DC = DC;
		motorSpeeds.m2_DC = DC;
		motorSpeeds.m3_DC = DC;
		motorSpeeds.m4_DC = DC;
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
			motorSpeeds.m1_DC 	+= rollPID;
			motorSpeeds.m4_DC 	-= rollPID;
			motorSpeeds.m2_DC	+= pitchPID;
			motorSpeeds.m3_DC 	-= pitchPID;

			// TODO: Yaw, elev PID.

			// Restrict duty cycle to max/min
			motorSpeeds.m1_DC = constrain(	motorSpeeds.m1_DC,
											passiveMIN - agroBOOL * agroSTEP,
											passiveMAX + agroBOOL * agroSTEP);
			motorSpeeds.m2_DC = constrain(	motorSpeeds.m2_DC,
											passiveMIN - agroBOOL * agroSTEP,
											passiveMAX + agroBOOL * agroSTEP);
			motorSpeeds.m3_DC = constrain(	motorSpeeds.m3_DC,
											passiveMIN - agroBOOL * agroSTEP,
											passiveMAX + agroBOOL * agroSTEP);
			motorSpeeds.m4_DC = constrain(	motorSpeeds.m4_DC,
											passiveMIN - agroBOOL * agroSTEP,
											passiveMAX + agroBOOL * agroSTEP);


			motor1.writeMicroseconds(motorSpeeds.m1_DC);
			motor2.writeMicroseconds(motorSpeeds.m2_DC);
			motor3.writeMicroseconds(motorSpeeds.m3_DC);
			motor4.writeMicroseconds(motorSpeeds.m4_DC);
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
