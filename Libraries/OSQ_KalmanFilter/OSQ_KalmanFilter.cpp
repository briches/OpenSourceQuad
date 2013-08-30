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

#include <OSQ_KalmanFilter.h>

void Kalman2D::Kalman2DPredict()
{
	// Store the updated x matrix
	x[0] = 1 * x[0] + 	1 * x[1] +	 U[0];
	x[1] = 1 * x[1]	+ 	U[1];

	P[0] = P[0]  + P[1];
	P[1] = P[1];
	P[2] = P[2]  + P[3];
	P[3] = P[3];

	P[0] = P[0];
	P[1] = P[0]  + P[1];
	P[2] = P[2];
	P[3] = P[2]  + P[3];
};


void Kalman2D::Kalman2DMeasure(double x_meas)
{
	Z = x_meas;

	Y = Z - x[0];

	S = R + P[0];

	K[0] = P[0] * 1 * 1/S + P[1] * 0 * 1/S;
	K[1] = P[2] * 1 * 1/S + P[3] * 0 * 1/S;

	K[0] = K[0] * Y;
	K[1] = K[1] * Y;


	// Store the updated x matrix
	x[0] += K[0];
	x[1] += K[1];

	double intermediate[2 * 2];

	intermediate[0] = K[0] * 1;
	intermediate[1] = K[0] * 0;
	intermediate[2] = K[1] * 1;
	intermediate[3] = K[1] * 0;

	intermediate[0] = I[0] - intermediate[0];
	intermediate[1] = I[1] - intermediate[1];
	intermediate[2] = I[2] - intermediate[2];
	intermediate[3] = I[3] - intermediate[3];

	P[0] = intermediate[0] * P[0] + intermediate[1] * P[2];
	P[1] = intermediate[0] * P[1] + intermediate[1] * P[3];
	P[2] = intermediate[2] * P[0] + intermediate[3] * P[2];
	P[3] = intermediate[2] * P[1] + intermediate[3] * P[3];

};


bool Kalman2D::KalmanInit_2D(double x_covar, double xdot_covar, double sensor_noise, double dt)
{
	x[0] = 0;
	x[1] = 0;

	//F = {1, 1, 0, 1};
	F[0] = 1;
	F[1] = dt;
	F[2] = 0;
	F[3] = 1;

	H[0] = 1;
	H[1] = 0;

	Z = 0;
	Y = 0;

	P[0] = x_covar;
	P[1] = 0;
	P[2] = 0;
	P[3] = xdot_covar;		// Initial uncertainty of velocity

	S = 0;
	K[0] = 0;
	K[1] = 0;

	I[0] = 1;
	I[1] = 0;
	I[2] = 0;
	I[3] = 1;

	U[0] = 0;
	U[1] = 0;

	R = sensor_noise;		// Covariance of the measurement

	return true;
};

void matrixInverse4x4(double final_mat[], double adj_mat[], double old_mat[])
{
	// Lost a lot of generality by including this function.
	// If more generality is needed (ie higher or lower ordered filters),
	// include some functions that invert matrices of different dim
	// or a general matrix inverter. Pretty hard.
	// Indexes are addressed by m00 ~~> m[4*row + column] = m[0]
	//							m23 ~~> m[4*2 + 3] 		  = m[11]
	adj_mat[0*4 +0] = old_mat[1*4 +2]*old_mat[2*4 +3]*old_mat[3*4 +1] - old_mat[1*4 +3]*old_mat[2*4 +2]*old_mat[3*4 +1] + old_mat[1*4 +3]*old_mat[2*4 +1]*old_mat[3*4 +2] - old_mat[1*4 +0]*old_mat[2*4 +3]*old_mat[3*4 +2] - old_mat[1*4 +2]*old_mat[2*4 +1]*old_mat[3*4 +3] + old_mat[1*4 +0]*old_mat[2*4 +2]*old_mat[3*4 +3];
	adj_mat[0*4 +1] = old_mat[0*4 +3]*old_mat[2*4 +2]*old_mat[3*4 +1] - old_mat[0*4 +2]*old_mat[2*4 +3]*old_mat[3*4 +1] - old_mat[0*4 +3]*old_mat[2*4 +1]*old_mat[3*4 +2] + old_mat[0*4 +1]*old_mat[2*4 +3]*old_mat[3*4 +2] + old_mat[0*4 +2]*old_mat[2*4 +1]*old_mat[3*4 +3] - old_mat[0*4 +1]*old_mat[2*4 +2]*old_mat[3*4 +3];
	adj_mat[0*4 +2] = old_mat[0*4 +2]*old_mat[1*4 +3]*old_mat[3*4 +1] - old_mat[0*4 +3]*old_mat[1*4 +2]*old_mat[3*4 +1] + old_mat[0*4 +3]*old_mat[1*4 +0]*old_mat[3*4 +2] - old_mat[0*4 +1]*old_mat[1*4 +3]*old_mat[3*4 +2] - old_mat[0*4 +2]*old_mat[1*4 +0]*old_mat[3*4 +3] + old_mat[0*4 +1]*old_mat[1*4 +2]*old_mat[3*4 +3];
	adj_mat[0*4 +3] = old_mat[0*4 +3]*old_mat[1*4 +2]*old_mat[2*4 +1] - old_mat[0*4 +2]*old_mat[1*4 +3]*old_mat[2*4 +1] - old_mat[0*4 +3]*old_mat[1*4 +0]*old_mat[2*4 +2] + old_mat[0*4 +1]*old_mat[1*4 +3]*old_mat[2*4 +2] + old_mat[0*4 +2]*old_mat[1*4 +0]*old_mat[2*4 +3] - old_mat[0*4 +1]*old_mat[1*4 +2]*old_mat[2*4 +3];
	adj_mat[1*4 +0] = old_mat[1*4 +3]*old_mat[2*4 +2]*old_mat[3*4 +0] - old_mat[1*4 +2]*old_mat[2*4 +3]*old_mat[3*4 +0] - old_mat[1*4 +3]*old_mat[2*4 +0]*old_mat[3*4 +2] + old_mat[1*4 +0]*old_mat[2*4 +3]*old_mat[3*4 +2] + old_mat[1*4 +2]*old_mat[2*4 +0]*old_mat[3*4 +3] - old_mat[1*4 +0]*old_mat[2*4 +2]*old_mat[3*4 +3];
	adj_mat[1*4 +1] = old_mat[0*4 +2]*old_mat[2*4 +3]*old_mat[3*4 +0] - old_mat[0*4 +3]*old_mat[2*4 +2]*old_mat[3*4 +0] + old_mat[0*4 +3]*old_mat[2*4 +0]*old_mat[3*4 +2] - old_mat[0*4 +0]*old_mat[2*4 +3]*old_mat[3*4 +2] - old_mat[0*4 +2]*old_mat[2*4 +0]*old_mat[3*4 +3] + old_mat[0*4 +0]*old_mat[2*4 +2]*old_mat[3*4 +3];
	adj_mat[1*4 +2] = old_mat[0*4 +3]*old_mat[1*4 +2]*old_mat[3*4 +0] - old_mat[0*4 +2]*old_mat[1*4 +3]*old_mat[3*4 +0] - old_mat[0*4 +3]*old_mat[1*4 +0]*old_mat[3*4 +2] + old_mat[0*4 +0]*old_mat[1*4 +3]*old_mat[3*4 +2] + old_mat[0*4 +2]*old_mat[1*4 +0]*old_mat[3*4 +3] - old_mat[0*4 +0]*old_mat[1*4 +2]*old_mat[3*4 +3];
	adj_mat[1*4 +3] = old_mat[0*4 +2]*old_mat[1*4 +3]*old_mat[2*4 +0] - old_mat[0*4 +3]*old_mat[1*4 +2]*old_mat[2*4 +0] + old_mat[0*4 +3]*old_mat[1*4 +0]*old_mat[2*4 +2] - old_mat[0*4 +0]*old_mat[1*4 +3]*old_mat[2*4 +2] - old_mat[0*4 +2]*old_mat[1*4 +0]*old_mat[2*4 +3] + old_mat[0*4 +0]*old_mat[1*4 +2]*old_mat[2*4 +3];
	adj_mat[2*4 +0] = old_mat[1*4 +1]*old_mat[2*4 +3]*old_mat[3*4 +0] - old_mat[1*4 +3]*old_mat[2*4 +1]*old_mat[3*4 +0] + old_mat[1*4 +3]*old_mat[2*4 +0]*old_mat[3*4 +1] - old_mat[1*4 +0]*old_mat[2*4 +3]*old_mat[3*4 +1] - old_mat[1*4 +0]*old_mat[2*4 +0]*old_mat[3*4 +3] + old_mat[1*4 +0]*old_mat[2*4 +1]*old_mat[3*4 +3];
	adj_mat[2*4 +1] = old_mat[0*4 +3]*old_mat[2*4 +1]*old_mat[3*4 +0] - old_mat[0*4 +1]*old_mat[2*4 +3]*old_mat[3*4 +0] - old_mat[0*4 +3]*old_mat[2*4 +0]*old_mat[3*4 +1] + old_mat[0*4 +0]*old_mat[2*4 +3]*old_mat[3*4 +1] + old_mat[0*4 +1]*old_mat[2*4 +0]*old_mat[3*4 +3] - old_mat[0*4 +0]*old_mat[2*4 +1]*old_mat[3*4 +3];
	adj_mat[2*4 +2] = old_mat[0*4 +1]*old_mat[1*4 +3]*old_mat[3*4 +0] - old_mat[0*4 +3]*old_mat[1*4 +0]*old_mat[3*4 +0] + old_mat[0*4 +3]*old_mat[1*4 +0]*old_mat[3*4 +1] - old_mat[0*4 +0]*old_mat[1*4 +3]*old_mat[3*4 +1] - old_mat[0*4 +1]*old_mat[1*4 +0]*old_mat[3*4 +3] + old_mat[0*4 +0]*old_mat[1*4 +0]*old_mat[3*4 +3];
	adj_mat[2*4 +3] = old_mat[0*4 +3]*old_mat[1*4 +1]*old_mat[2*4 +0] - old_mat[0*4 +1]*old_mat[1*4 +3]*old_mat[2*4 +0] - old_mat[0*4 +3]*old_mat[1*4 +0]*old_mat[2*4 +1] + old_mat[0*4 +0]*old_mat[1*4 +3]*old_mat[2*4 +1] + old_mat[0*4 +1]*old_mat[1*4 +0]*old_mat[2*4 +3] - old_mat[0*4 +0]*old_mat[1*4 +0]*old_mat[2*4 +3];
	adj_mat[3*4 +0] = old_mat[1*4 +2]*old_mat[2*4 +1]*old_mat[3*4 +0] - old_mat[1*4 +0]*old_mat[2*4 +2]*old_mat[3*4 +0] - old_mat[1*4 +2]*old_mat[2*4 +0]*old_mat[3*4 +1] + old_mat[1*4 +0]*old_mat[2*4 +2]*old_mat[3*4 +1] + old_mat[1*4 +0]*old_mat[2*4 +0]*old_mat[3*4 +2] - old_mat[1*4 +0]*old_mat[2*4 +1]*old_mat[3*4 +2];
	adj_mat[3*4 +1] = old_mat[0*4 +1]*old_mat[2*4 +2]*old_mat[3*4 +0] - old_mat[0*4 +2]*old_mat[2*4 +1]*old_mat[3*4 +0] + old_mat[0*4 +2]*old_mat[2*4 +0]*old_mat[3*4 +1] - old_mat[0*4 +0]*old_mat[2*4 +2]*old_mat[3*4 +1] - old_mat[0*4 +1]*old_mat[2*4 +0]*old_mat[3*4 +2] + old_mat[0*4 +0]*old_mat[2*4 +1]*old_mat[3*4 +2];
	adj_mat[3*4 +2] = old_mat[0*4 +2]*old_mat[1*4 +1]*old_mat[3*4 +0] - old_mat[0*4 +1]*old_mat[1*4 +2]*old_mat[3*4 +0] - old_mat[0*4 +2]*old_mat[1*4 +0]*old_mat[3*4 +1] + old_mat[0*4 +0]*old_mat[1*4 +2]*old_mat[3*4 +1] + old_mat[0*4 +1]*old_mat[1*4 +0]*old_mat[3*4 +2] - old_mat[0*4 +0]*old_mat[1*4 +0]*old_mat[3*4 +2];
	adj_mat[3*4 +3] = old_mat[0*4 +1]*old_mat[1*4 +2]*old_mat[2*4 +0] - old_mat[0*4 +2]*old_mat[1*4 +0]*old_mat[2*4 +0] + old_mat[0*4 +2]*old_mat[1*4 +0]*old_mat[2*4 +1] - old_mat[0*4 +0]*old_mat[1*4 +2]*old_mat[2*4 +1] - old_mat[0*4 +1]*old_mat[1*4 +0]*old_mat[2*4 +2] + old_mat[0*4 +0]*old_mat[1*4 +0]*old_mat[2*4 +2];

	double determinant = matrixDeterminant4x4(old_mat);

	matrixScale(4,4,final_mat,determinant,adj_mat);
};


double matrixDeterminant4x4(double m[])
{
	double determinant =
	m[0*4 +3]*m[1*4 +2]*m[2*4 +1]*m[3*4 +0] - m[0*4 +2]*m[1*4 +3]*m[2*4 +1]*m[3*4 +0] - m[0*4 +3]*m[1*4 +1]*m[2*4 +2]*m[3*4 +0] + m[0*4 +1]*m[1*4 +3]*m[2*4 +2]*m[3*4 +0]+
	m[0*4 +2]*m[1*4 +1]*m[2*4 +3]*m[3*4 +0] - m[0*4 +1]*m[1*4 +2]*m[2*4 +3]*m[3*4 +0] - m[0*4 +3]*m[1*4 +2]*m[2*4 +0]*m[3*4 +1] + m[0*4 +2]*m[1*4 +3]*m[2*4 +0]*m[3*4 +1]+
	m[0*4 +3]*m[1*4 +0]*m[2*4 +2]*m[3*4 +1] - m[0*4 +0]*m[1*4 +3]*m[2*4 +2]*m[3*4 +1] - m[0*4 +2]*m[1*4 +0]*m[2*4 +3]*m[3*4 +1] + m[0*4 +0]*m[1*4 +2]*m[2*4 +3]*m[3*4 +1]+
	m[0*4 +3]*m[1*4 +1]*m[2*4 +0]*m[3*4 +2] - m[0*4 +1]*m[1*4 +3]*m[2*4 +0]*m[3*4 +2] - m[0*4 +3]*m[1*4 +0]*m[2*4 +1]*m[3*4 +2] + m[0*4 +0]*m[1*4 +3]*m[2*4 +1]*m[3*4 +2]+
	m[0*4 +1]*m[1*4 +0]*m[2*4 +3]*m[3*4 +2] - m[0*4 +0]*m[1*4 +1]*m[2*4 +3]*m[3*4 +2] - m[0*4 +2]*m[1*4 +1]*m[2*4 +0]*m[3*4 +3] + m[0*4 +1]*m[1*4 +2]*m[2*4 +0]*m[3*4 +3]+
	m[0*4 +2]*m[1*4 +0]*m[2*4 +1]*m[3*4 +3] - m[0*4 +0]*m[1*4 +2]*m[2*4 +1]*m[3*4 +3] - m[0*4 +1]*m[1*4 +0]*m[2*4 +2]*m[3*4 +3] + m[0*4 +0]*m[1*4 +1]*m[2*4 +2]*m[3*4 +3];


	return determinant;
};

/*===============================================================================
	Functions below this point are written by the AeroQuad team
	----------------------------------------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////
//  Vector Dot Product
//  Compute the dot product of vectors a and b with length 3
//  Place result in vector C
//
//  Call as: vectorDotProduct(c, a, b)
////////////////////////////////////////////////////////////////////////////////
double vectorDotProduct(int length, double vector1[], double vector2[])
{
  double dotProduct = 0;
  //int   i;

  for (int i = 0; i < length; i++)
  {
  dotProduct += vector1[i] * vector2[i];
  }

  return dotProduct;
};

////////////////////////////////////////////////////////////////////////////////
//  Vector Cross Product
//  Compute the cross product of vectors a and b with length 3
//  Place result in vector C
//
//  Call as: vectorCrossProduct(c, a, b)
////////////////////////////////////////////////////////////////////////////////
void vectorCrossProduct(double vectorC[3], double vectorA[3], double vectorB[3])
{
  vectorC[0] = (vectorA[1] * vectorB[2]) - (vectorA[2] * vectorB[1]);
  vectorC[1] = (vectorA[2] * vectorB[0]) - (vectorA[0] * vectorB[2]);
  vectorC[2] = (vectorA[0] * vectorB[1]) - (vectorA[1] * vectorB[0]);
};

////////////////////////////////////////////////////////////////////////////////
//  Multiply a vector by a scalar
//  Mulitply vector a with length m by a scalar
//  Place result in vector b
//
//  Call as: vectorScale(m, b, a, scalar)
////////////////////////////////////////////////////////////////////////////////
void vectorScale(int length, double scaledVector[], double inputVector[], double scalar)
{
  for (int i = 0; i < length; i++)
  {
   scaledVector[i] = inputVector[i] * scalar;
  }
};

////////////////////////////////////////////////////////////////////////////////
//  Compute sum of 2 vectors
//  Add vector a to vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorAdd(m, c, b, a)
////////////////////////////////////////////////////////////////////////////////
void vectorAdd(int length, double vectorC[], double vectorA[], double vectorB[])
{
  for(int i = 0; i < length; i++)
  {
     vectorC[i] = vectorA[i] + vectorB[i];
  }
};

////////////////////////////////////////////////////////////////////////////////
//  Compute difference of 2 vectors
//  Subtract vector a from vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorSubtract(m, c, b, a)
////////////////////////////////////////////////////////////////////////////////
void vectorSubtract(int length, double vectorC[], double vectorA[], double vectorB[])
{
  for(int i = 0; i < length; i++)
  {
     vectorC[i] = vectorA[i] - vectorB[i];
  }
};

////////////////////////////////////////////////////////////////////////////////
//  Matrix Multiply
//  Multiply matrix A times matrix B, matrix A dimension m x n, matrix B dimension n x p
//  Result placed in matrix C, dimension m x p
//
//  Call as: matrixMultiply(m, n, p, C, A, B)
////////////////////////////////////////////////////////////////////////////////
void matrixMultiply(int aRows, int aCols_bRows, int bCols, double matrixC[], double matrixA[], double matrixB[])
{
  for (int i = 0; i < aRows * bCols; i++)
  {
    matrixC[i] = 0.0;
  }

  for (int i = 0; i < aRows; i++)
  {
    for(int j = 0; j < aCols_bRows; j++)
    {
      for(int k = 0;  k < bCols; k++)
      {
       matrixC[i * bCols + k] += matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
      }
    }
  }
};

////////////////////////////////////////////////////////////////////////////////
//  Matrix Addition
//  Add matrix A to matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixAdd(m, n, C, A, B)
////////////////////////////////////////////////////////////////////////////////
void matrixAdd(int rows, int cols, double matrixC[], double matrixA[], double matrixB[])
{
  for (int i = 0; i < rows * cols; i++)
  {
    matrixC[i] = matrixA[i] + matrixB[i];
  }
};

////////////////////////////////////////////////////////////////////////////////
//  Matrix Subtraction
//  Subtract matrix A from matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixSubtract(m, n, C, A, B)
////////////////////////////////////////////////////////////////////////////////
void matrixSubtract(int rows, int cols, double matrixC[], double matrixA[], double matrixB[])
{
  for (int i = 0; i < rows * cols; i++)
  {
    matrixC[i] = matrixA[i] - matrixB[i];
  }
};


////////////////////////////////////////////////////////////////////////////////
//  Matrix Scaling
//  Scale matrix A, dimensions m x n, by a scaler, S
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixScale(m, n, C, S, B)
////////////////////////////////////////////////////////////////////////////////
void matrixScale(int rows, int cols, double matrixC[], double scaler, double matrixA[])
{
  for (int i = 0; i < rows * cols; i++)
  {
    matrixC[i] = scaler * matrixA[i];
  }
};

////////////////////////////////////////////////////////////////////////////////
//  3 x 3 Matrix Transpose
//  Compute 3 x 3 Transpose of A
//  Result placed in matrix C, dimension 3 x 3
//
//  Call as: Transpose3x3(C, A)
////////////////////////////////////////////////////////////////////////////////

void matrixTranspose3x3(double matrixC[9], double matrixA[9])
{
  matrixC[0] = matrixA[0];
  matrixC[1] = matrixA[3];
  matrixC[2] = matrixA[6];
  matrixC[3] = matrixA[1];
  matrixC[4] = matrixA[4];
  matrixC[5] = matrixA[7];
  matrixC[6] = matrixA[2];
  matrixC[7] = matrixA[5];
  matrixC[8] = matrixA[8];
};

////////////////////////////////////////////////////////////////////////////////
//  3 x 3 Matrix Inverse
//  Compute 3 x 3 Inverse of A
//  Result placed in matrix C, dimension 3 x 3
//
//  Call as: Inverse3x3(C, A)
////////////////////////////////////////////////////////////////////////////////
void matrixInverse3x3(double matrixC[9], double matrixA[9])
{

  double det;
  double transposeA[9];
  double minors[9];
  double transposeMinors[9];

  det = matrixA[0] * (matrixA[4] * matrixA[8] - matrixA[5] * matrixA[7]) -
        matrixA[1] * (matrixA[3] * matrixA[8] - matrixA[5] * matrixA[6]) +
        matrixA[2] * (matrixA[3] * matrixA[7] - matrixA[4] * matrixA[6]);

  matrixTranspose3x3(transposeA, matrixA);

  minors[0] = matrixA[4] * matrixA[8] - matrixA[5] * matrixA[7];
  minors[1] = matrixA[5] * matrixA[6] - matrixA[3] * matrixA[8];
  minors[2] = matrixA[3] * matrixA[7] - matrixA[4] * matrixA[6];
  minors[3] = matrixA[2] * matrixA[7] - matrixA[1] * matrixA[8];
  minors[4] = matrixA[0] * matrixA[8] - matrixA[2] * matrixA[6];
  minors[5] = matrixA[1] * matrixA[6] - matrixA[0] * matrixA[7];
  minors[6] = matrixA[1] * matrixA[5] - matrixA[2] * matrixA[4];
  minors[7] = matrixA[2] * matrixA[3] - matrixA[0] * matrixA[5];
  minors[8] = matrixA[0] * matrixA[4] - matrixA[1] * matrixA[3];

  matrixTranspose3x3(transposeMinors, minors);

  det = 1/det;

  matrixScale(3,3, matrixC, det, transposeMinors);
};
