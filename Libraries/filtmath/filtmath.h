/*===============================================================================
	filtmath.cpp

	Performs cheby2 filter on a set of data, with given
	input A and B coefficents

	Author: Brandon Riches

	Matrix math functions from AeroQuad library AQ_Math
	with license

	AeroQuad v3.0.1 - February 2012
	www.AeroQuad.com
	Copyright (c) 2012 Ted Carancho.  All rights reserved.
	An Open Source Arduino based multicopter.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
	----------------------------------------------------------------------------*/

#ifndef FILTMATH_H_INCLUDED
#define FILTMATH_H_INCLUDED

void filtcheby2(double A[], double B[], double x[], double y[]);

float vectorDotProduct(int length, float vector1[], float vector2[]);

void vectorCrossProduct(float vectorC[3], float vectorA[3], float vectorB[3]);

void vectorScale(int length, float scaledVector[], float inputVector[], float scalar);

void vectorAdd(int length, float vectorC[], float vectorA[], float vectorB[]);

void vectorSubtract(int length, float vectorC[], float vectorA[], float vectorB[]);

void matrixMultiply(int aRows, int aCols_bRows, int bCols, float matrixC[], float matrixA[], float matrixB[]);

void matrixAdd(int rows, int cols, float matrixC[], float matrixA[], float matrixB[]);

void matrixSubtract(int rows, int cols, float matrixC[], float matrixA[], float matrixB[]);

void matrixScale(int rows, int cols, float matrixC[], float scaler, float matrixA[]);

void matrixTranspose3x3(float matrixC[9], float matrixA[9]);

void matrixInverse3x3(float matrixC[9], float matrixA[9]);

#endif // FILTMATH_H_INCLUDED
