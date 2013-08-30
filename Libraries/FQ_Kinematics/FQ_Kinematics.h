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
#include <Wire.h>
#include <I2C.h>

void kinematicEvent(int eventType,
					struct kinematicData *data,
					class SENSORLIB_accel *accel,
					class SENSORLIB_mag *mag,
					class SENSORLIB_gyro *gyro,
					struct fourthOrderData *fourthOrderXAXIS,
					struct fourthOrderData *fourthOrderYAXIS,
					struct fourthOrderData *fourthOrderZAXIS);

double computeFourthOrder(double currentInput, struct fourthOrderData *filterParameters);

void setupFourthOrder(	struct fourthOrderData *fourthOrderXAXIS,
						struct fourthOrderData *fourthOrderYAXIS,
						struct fourthOrderData *fourthOrderZAXIS);



#endif // KINEMATICS_H_INCLUDED
