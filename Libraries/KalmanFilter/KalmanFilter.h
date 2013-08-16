/**=========================================================================

	KalmanFilter.h
	August 2013

	A library designed to easily implement 2D or 4D state Kalman filters
    -----------------------------------------------------------------------*/

#ifndef KALMANFILTER_H_INCLUDED
#define KALMANFILTER_H_INCLUDED


typedef struct
{
	int xdim[2] = {2, 1};
	double x[2*1] = {0,
					  0};				// State matrix

	int Fdim[2] = {2, 2};
	double F[2*2] = {1, 1,
					  0, 1};			// State transition matrix

	int Hdim[2] = {1, 2};
	double H[1*2] = {1, 0};			// Measurement function

	double Z;							// Measurement

	double y;							// Error

	int Pdim[2] = {2, 2};				// Covariance matrix
	double P[2*2] = {50, 0,
					  0, 50};

	double S;

	int Kdim[2] = {1, 2};
	double K[1*2] = {0,				// Kalman gain
					  0};

	double I[2*2] = {1, 0,
					  0, 1};			// Identity

	double Udim[2] = {2, 1};			// External motion
	double U[2*1] = {0,
					  0};

	double R;


} Kalman2D_struct;



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




#endif // KALMANFILTER_H_INCLUDED
