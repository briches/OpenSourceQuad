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
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

void filtcheby2(double A[], double B[], double x[], double y[],int SIZE_A, int SIZE_B, int SIZE_X);

double matrixDeterminant4x4(double m[]);

void matrixInverse4x4(double final_mat[], double adj_mat[], double old_mat[]);

void sparse_matrix(int rows[], int cols[], double data[], double sparse[], double A[], int nfilt);

double vectorDotProduct(int length, double vector1[], double vector2[]);

void vectorCrossProduct(double vectorC[3], double vectorA[3], double vectorB[3]);

void vectorScale(int length, double scaledVector[], double inputVector[], double scalar);

void vectorAdd(int length, double vectorC[], double vectorA[], double vectorB[]);

void vectorSubtract(int length, double vectorC[], double vectorA[], double vectorB[]);

void matrixMultiply(int aRows, int aCols_bRows, int bCols, double matrixC[], double matrixA[], double matrixB[]);

void matrixAdd(int rows, int cols, double matrixC[], double matrixA[], double matrixB[]);

void matrixSubtract(int rows, int cols, double matrixC[], double matrixA[], double matrixB[]);

void matrixScale(int rows, int cols, double matrixC[], double scaler, double matrixA[]);

void matrixTranspose3x3(double matrixC[9], double matrixA[9]);

void matrixInverse3x3(double matrixC[9], double matrixA[9]);

#endif // FILTMATH_H_INCLUDED
