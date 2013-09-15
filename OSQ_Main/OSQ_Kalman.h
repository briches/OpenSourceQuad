/*=====================================================================
	OSQ_Kalman library
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Andrew Coulthard
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
	
#include <iostream>
#include <string.h>
using namespace std;

double P0 = 1000;
double P1 = 0;
double P2 = 0;
double P3 = 1000;

#define measurement_noise 1

double x1 = 0;
double x2 = 0;

double KalmanUpdate(float z) {

/* Measurement Update */

	// y = z - Hx

	float y;
	y = z - x1;

	// S = H*P*HT + measurement_noise

	float S;
	S = P0 + measurement_noise;

	// K = P*HT*S^-1

	float K[2];
	K[0] = P0 * (1 / S);
	K[1] = P2 * (1 / S);

	// X^ = X + K*y

	float Ky[2];
	Ky[0] = K[0]*y;
	Ky[1] = K[1]*y;
	x1 += Ky[0];
	x2 += Ky[1];

	// P^ = (I - K*H) * P

	float a;
	a = P0 * (1 - K[0]);
	float b;
	b = P1 * (1 - K[0]);
	P2 += P0 * (0 - K[1]);
	P3 += P1 * (0 - K[1]);
	P0 = a;
	P1 = b;

/* Prediction */

	// x

	x1 += x2;

	// P

	P0 += P1 + P2 + P3;
	P1 += P3;
	P2 += P3;
	
	return(x1);
}

double KalmanSpeedReturn() {
return(x2);
}

