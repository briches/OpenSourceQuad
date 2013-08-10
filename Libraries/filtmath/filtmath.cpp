/*===============================================================================
	filtmath.cpp

	This library contains a multitude of math related functions,
	including one that performs a cheby2 filter on a set of data, with given
	input A and B coefficents

	The difference equation for a nth order DF-2 IIR filter can be written as:

	v(n) = x(n) - a1 * v(n-1) - a2 * v(n-2) - ... - an * v(0)

	y(n) = b0 * v(n) + b1 * v(n-1) + ... + bn * v(0)

	Author: Brandon Riches

	Matrix math functions from AeroQuad library AQ_Math
	under the GNU General Public License

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

#include "filtmath.h"
#include <math.h>

void filtcheby2(double A[], double B[], double x[], double y[],int SIZE_A, int SIZE_B, int SIZE_X)
{
	int		na,			// The number of elements in the B array
			nb,			// The number of elements in the A array
			len,		// The number of elements in the x array
			nfilt,		// The max between na and nb
			nfact;		// The length of edge transients

	double extra[38];
	double flip[38];
	double w[38];	// Intermediate, used to filter backwards

	// Get size of A and B
	na = SIZE_A;
	nb = SIZE_B;
	len = SIZE_X;

	// Debug
	/*
	Serial.println(na);
	Serial.println(nb);
	Serial.println(len);
	*/

	// nfilt is the largest of na and nb
	if (na > nb)
	{
		nfilt = na;
	} else
	{
		nfilt = nb;
	}

	// Length of edge transients
	nfact = 3*(nfilt - 1);

	// Debug
	/*
	Serial.println(nfilt);
	Serial.println(nfact);
	*/


	/// Remove possible DC offsets and pad with zeros
	if (nb < nfilt)
	{
		B[nfilt] = 0;
	}
	if (na < nfilt)
	{
		A[nfilt] = 0;
	}

	// If you arent using a fourth order filter,
	// ie your nfilt is 5,
	// This code block will NOT work for you :(

	// On the other hand, if you have nfilt = 5, fantastic. This works. As long as size(y) = 2*nfact + size(x)
	for (int i = 0; i< nfact; i++)
	{
		extra[i] = 2*x[0] - x[12 - i];
	}
	for (int i = 12; i<= 25; i++)
	{
		extra[i] = x[i-12];
	}
	for (int i = 26; i <= 37;i++)
	{
		extra[i] = (2*x[13]) - x[13 - (i - 25)];
	}

	//Debug
	/*
	for (int i = 0; i < 38; i++)
	{
		Serial.println(extra[i]);
	}
	*/

	///!  @Main filter structure

	// Fill the first "na" values of w with the values of y
	for (int i = 0; i < na; i++)
	{
		w[i] = extra[i];
	}

	/// Implement the difference equation forwards
	for (int n =  na; n < 38; n++)
	{
		w[n] = 	B[0] * extra[n] + B[1] * extra[n-1] + B[2] * extra[n-2] + B[3] * extra[n-3] + B[4] * extra[n-4]
								- A[1] * w[n-1] 	- A[2] * w[n-2]		- A[3] * w[n-3] 	- A[4] * w[n-4];


	}

	// Remove extrapolated bits
	for (int i = 0; i<14; i++)
	{
		y[i] = w[i+12];
	}

	//Debug

	for (int i = 0; i < 38; i++)
	{
		Serial.println(w[i]);
	}


				/// Implement the difference equation backwards.

				/*
				// Reverse the array
				for (int i =0; i< 38; i++)
				{
					flip[38-i] = extra[i];
				}

				// Fill the first "na" values of w with the values of y
				for (int i = 0; i < na; i++)
				{
					w[i] = flip[i];
				}

				for (int n = 5; n < 38; n++)
				{
					for (int k = 1; k< na; k++)
					{
						w[n] = A[k]*w[n-k] + flip[n];
					}

				}
				for (int i = na; i< 38; i++)
				{
					for (int k = 1; k< nb; k++)
					{
						flip[i] += B[k] * w[i-k];
					}
				}

				// Reverse it again
				for (int i =0; i< 38; i++)
				{
					extra[38-i] = flip[i];
				}

				// Remove extrapolated bits
				for (int i = 0; i<14; i++)
				{
					y[i] = extra[i+12];
				}
				*/

	// Debug
	/*
	for (int i =0; i<14; i++)
	{
		Serial.println(y[i]);
	}
	*/

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

void sparse_matrix(int rows[], int cols[], double data[], double sparse[], double A[], int nfilt)
{
	for (int i = 0; i <= nfilt-2; i++)
	{
		rows[i] = i;
	}
	for (int i = nfilt-1; i <= ((2*nfilt) - 4); i++)
	{
		rows[i] = i - 3;
	}
	for (int i = ((2*nfilt)-3); i<= (3*nfilt - 6); i++)
	{
		rows[i] = i - 7;
	}


	// Fill the column array with the addresses
	for (int i = 0; i <= nfilt-2; i++)
	{
		cols[i] = 0;
	}
	for (int i = nfilt-1; i <= ((2*nfilt) - 4); i++)
	{
		cols[i] = i - 3;
	}
	for (int i = ((2*nfilt)-3); i<= (3*nfilt - 6); i++)
	{
		cols[i] = i - 6;
	}



	// Rows and columns generated, generate the data
	data[0] = 1+ A[1];
	for (int i = 1; i <= nfilt-2; i++)
	{
		data[i] = A[i+1];
	}
	for (int i = nfilt-1; i <= ((2*nfilt) - 4); i++)
	{
		data[i] = 1;
	}
	for (int i = ((2*nfilt)-3); i<= (3*nfilt - 6); i++)
	{
		data[i] = -1;
	}

	int sparse_size1d = nfilt - 1;
	// Fill a 1D (2D) matrix with the data.
	for (int i = 0; i < ((3 * nfilt) - 5);i++)
	{	// sparse[col][row]
		int my_row = rows[i];
		int my_column = cols[i];

		sparse[sparse_size1d*my_row + my_column] = data[i];
	}


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
