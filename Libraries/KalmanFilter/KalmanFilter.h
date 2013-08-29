/**=========================================================================

	KalmanFilter.h
	August 2013

	A library designed to easily implement 2D or 4D state Kalman filters
    -----------------------------------------------------------------------*/

#ifndef KALMANFILTER_H_INCLUDED
#define KALMANFILTER_H_INCLUDED

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




#endif // KALMANFILTER_H_INCLUDED
