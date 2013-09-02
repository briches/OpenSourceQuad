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

#include <OSQ_SENSORLIB.h>
#include <Wire.h>
#include <I2C.h>

#define Pi  			(3.14159265359F)			// Its pi.

#define ORDER 4

// Comment out one of these defines to select the coefficent set to use.
// Remember that Wst is a fraction of the Nyquist frequency, Nq = Ws/2
// "BrandonCoeffs" uses n = 4, r = 10, Wc = 0.1
// AeroQuad filter uses n = 4, r = 60, Wc = 12.5/50

#define AeroQuadCoeff
// #define BrandonCoeffs

#ifdef AeroQuadCoeff
	#define ORDER 4
	#define _b0  0.001893594048567
	#define _b1 -0.002220262954039
	#define _b2  0.003389066536478
	#define _b3 -0.002220262954039
	#define _b4  0.001893594048567

	#define _a0  1
	#define _a1 -3.362256889209355
	#define _a2  4.282608240117919
	#define _a3 -2.444765517272841
	#define _a4  0.527149895089809
#endif

#ifdef BrandonCoeffs
	#define ORDER 4
	#define _b0  0.267411759560506
	#define _b1 -1.018535973364803
	#define _b2  1.503499251793105
	#define _b3 -1.018535973364804
	#define _b4  0.267411759560506

	#define _a0  1.000000000000000
	#define _a1 -3.561271663800053
	#define _a2  4.788976593687705
	#define _a3 -2.881867760094174
	#define _a4  0.655413654391032
#endif

struct fourthOrderData
{
  double  inputTm1,  inputTm2,  inputTm3,  inputTm4;
  double outputTm1, outputTm2, outputTm3, outputTm4;
};

/*=========================================================================
    Kinematics Data Type
    -----------------------------------------------------------------------*/
struct kinematicData
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

};


#define XAXIS   (0)
#define YAXIS	(1)
#define ZAXIS	(2)

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
