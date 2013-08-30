////========================================================================
	/*//*//*   FreeQuad   *//*//*
	FQ_Motors.cpp

	Author		: Brandon Riches
	Date		: August 2013

	Library designed to interface with and control brushless motor escs
    -----------------------------------------------------------------------*/


#ifndef FQ_MOTORS_H_INCLUDED
#define FQ_MOTORS_H_INCLUDED

#include <QuadGlobalDefined.h>
#include <Servo.h>

struct motorControl_t
{
	double m1_DC;
	double m2_DC;
	double m3_DC;
	double m4_DC;
};


class FQ_MotorControl
{
	public:
		FQ_MotorControl(int num = 4);
		Servo           	motor1;		// Motor 1
		Servo            	motor2;		// Motor 2
		Servo            	motor3;		// Motor 3
		Servo             	motor4;		// Motor 4

		motorControl_t		motorControl;

		// Function ideas //
		void calibrateESC(int numESC = 4);
		void motorDISARM();
		void startMotors(int startPercentDC);
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
