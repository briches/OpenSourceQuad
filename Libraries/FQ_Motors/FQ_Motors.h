////========================================================================
	/*//*//*   FreeQuad   *//*//*
	QuadGlobalDefined.h
	August 2013

	A comprehensive functional list of all (most) variables used in any of
	the libraries written by our team.
	Currently, these include:
		Quadcopter.h
		SENSORLIB.h
		OseppGyro.h
    -----------------------------------------------------------------------*/


#ifndef FQ_MOTORS_H_INCLUDED
#define FQ_MOTORS_H_INCLUDED

static int NUM_MOTORS = 4;
static bool MOTORS_ARMED = false;

// Function ideas //
void calibrateESC(int numESC = 4);

void motorStop();

void armMotors();

#endif // FQ_MOTORS_H_INCLUDED
