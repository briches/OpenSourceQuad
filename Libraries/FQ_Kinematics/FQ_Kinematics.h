//***************************************************/
/*//*//*   FreeQuad   *//*//*

Abstracts out the kinematic calculations from the other
libraries and main files.

Author  : Brandon Riches
Date    :	August 2013


******************************************************/

#ifndef KINEMATICS_H_INCLUDED
#define KINEMATICS_H_INCLUDED

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <FQ_Quadcopter.h>
#include <FQ_QuadGlobalDefined.h>
#include <FQ_SENSORLIB.h>
#include <FQ_OseppGyro.h>


/*
typedef struct kinematicData_s
{
	double pitch,
			roll,
			yaw,

			io_ax,
			io_ay,
			io_az,
			io_wx,
			io_wy,
			io_wz,

			pitch_gyro,
			roll_gyro,
			yaw_gyro,

			yaw_mag;

	unsigned long timestamp;

} kinematicData;*/

void kinematicEvent(int eventType,
					struct kinematicData data,
					SENSORLIB_accel accel,
					SENSORLIB_mag mag,
					OseppGyro gyro,
					fourthOrderData fourthOrderXAXIS,
					fourthOrderData fourthOrderYAXIS,
					fourthOrderData fourthOrderZAXIS);

float computeFourthOrder(float currentInput, struct fourthOrderData *filterParameters);

void setupFourthOrder(	fourthOrderData fourthOrderXAXIS,
						fourthOrderData fourthOrderYAXIS,
						fourthOrderData fourthOrderZAXIS);



#endif // KINEMATICS_H_INCLUDED
