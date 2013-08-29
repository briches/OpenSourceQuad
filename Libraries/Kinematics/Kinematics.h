#ifndef KINEMATICS_H_INCLUDED
#define KINEMATICS_H_INCLUDED

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Quadcopter.h>
#include <QuadGlobalDefined.h>
#include <SENSORLIB.h>
#include <OseppGyro.h>


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

void setupFourthOrder(void);



#endif // KINEMATICS_H_INCLUDED
