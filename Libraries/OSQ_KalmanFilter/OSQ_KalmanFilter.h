/*=====================================================================
	OSQ_KalmanFilter library
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: August 2013
	License		: GNU Public License

	This library designed to easily implement 2D or 4D state vector
	Kalman filters.

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

#ifndef OSQ_KALMANFILTER_H_INCLUDED
#define OSQ_KALMANFILTER_H_INCLUDED

class Kalman2D
{
	public:

		double x[2];			// State matrix
		double F[4];			// State transition matrix
		double H[1*2];			// Measurement function
		double Z;				// Measurement
		double Y;				// Error
		double P[4];			// Covariance matrix
		double S;
		double K[2*1];			// Kalman gain
		double I[2*2];			// Identity
		double U[2*1];			// External motion
		double R;				// Measurement noise


		void Kalman2DPredict(void);

		void Kalman2DMeasure(double x_meas);

		bool KalmanInit_2D(double x_covar, double xdot_covar, double sensor_noise, double dt);



};

double matrixDeterminant4x4(double m[]);

void matrixInverse4x4(double final_mat[], double adj_mat[], double old_mat[]);

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




#endif // OSQ_KALMANFILTER_H_INCLUDED
