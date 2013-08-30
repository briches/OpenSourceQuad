/*=====================================================================
	OSQ_Kinematics library
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: August 2013
	License		: GNU Public License

	This library implements sensor measuments to update the measured state
	of the craft.

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

#ifndef OSQ_KINEMATICS_H_INCLUDED
#define OSQ_KINEMATICS_H_INCLUDED

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <OSQ_Quadcopter.h>
#include <OSQ_QuadGlobalDefined.h>
#include <OSQ_SENSORLIB.h>
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
