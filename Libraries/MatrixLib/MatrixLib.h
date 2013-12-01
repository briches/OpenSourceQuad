/* Matrix library for implementing matrix math with low resource consumption */

/**//** Included functions: /**//**/
// typedef Matrix_t
// constructor Matrix_t(int nrows, int ncols)
// destructor ~Matrix_t()
// void Matrix_t :: set(int row, int col, double value)

#ifndef OSQ_MATRIXLIB_H_INCLUDED
#define OSQ_MATRIXLIB_H_INCLUDED

#include <stdlib.h>
#include <iostream>
#include <math.h>
using namespace std;

#if ARDUINO > 100
	#include "Arduino.h"
#endif

#define NO_ERROR					0x00
#define MATRIX_INPUT_DIM_ERR		0x02
#define MATRIX_OUTPUT_DIM_ERR		0x04
#define MATRIX_POSITIVE_DEFINE_ERR  0x06
#define NON_SQUARE_MATRIX_ERR		0x08



typedef struct Matrix_t
{
	// Matrices are addressed in row major format, ie: Matrix(row, col) = a;

	int rows, cols;
	double* pdata;

	void set(int row, int col, double value);

	Matrix_t(int nrows, int ncols); // Constructor

	~Matrix_t(); // Destructor

}Matrix;

void Matrix_t :: set(int row, int col, double value)
{
	pdata[row*cols + col] = value;
};


/* Constructor */
Matrix_t :: Matrix_t(int nrows, int ncols)
{
	rows = nrows;
	cols = ncols;

	// Allocate memory dynamically for the matrix.
	pdata =  (double*) calloc(rows * cols, sizeof(double));
};

/* Destructor */
Matrix_t :: ~Matrix_t()
{
	free (pdata);
};

/*----------------------------------------
	Other Functions
	------------------------------------*/
uint8_t matrixMul(Matrix_t* dest, Matrix_t* A, Matrix_t* B);
uint8_t cholInv(Matrix_t* dest, Matrix_t* A);

/*=======================================================
	Main Matrix Methods
	Not included in Matrix type
	---------------------------------------------------*/
uint8_t cholInv(Matrix_t* dest, Matrix_t* A)
{
	/** uint8_t error = cholInv(&dest, &A); **/
	// Inverts a matrix A using cholesky decomposition
	// dest will store the inverse of A
	// A must be square and positive defined, ie A00 > 0

	// Check for positive defined matrix
	if(A->pdata[0] < 0)
		return MATRIX_POSITIVE_DEFINE_ERR;

	// Check for square matrix
	if(A->cols != A->rows)
		return NON_SQUARE_MATRIX_ERR;

	// Check that dest is same size as A
	if(A-> rows != dest->rows && A->cols != dest->cols)
		return MATRIX_INPUT_DIM_ERR;

	// Both A and dest are square
	int n = A->cols;
	Matrix destInv(n,n), destTr(n,n);

    for(int j = 0; j < n; j++) // Columns of A
	{
		for(int i = 0; i < n; i++) // Rows of A
		{
			double sum1 = 0, sum2 = 0;

			if(j > 0)
			{ // j, i = 3
				for(int k = 0; k <= j-1; k++)
				{
					sum1 += (dest->pdata[k + j*n])*(dest->pdata[k + j*n]);
				}
			}

			dest->pdata[j+j*n] = sqrt(A->pdata[j+j*n] - sum1);

			if(i > j)
			{
				if(j > 0)
				{
					for(int k = 0; k <= j-1; k++)
					{
						sum2 += dest->pdata[i*n + k] * dest->pdata[j*n + k];
					}
				}
				dest->pdata[i*n + j] = (A->pdata[i*n + j] - sum2) / dest->pdata[j+j*n];
			}
		}
	}

	// Populate the upper-triangular matrix
	for(int i = 0; i < n; i++) // rows of dest
	{
		for(int j = 0; j < n; j++)
		{
			destTr.pdata[j*n + i] = dest->pdata[i*n + j];
		}
	}

	for(int i = 0; i<n; i++)
	{
		destInv.pdata[i*n + i] = 1;
	}

	for(int column = n-1; column >= 0; column--) // The column'th column of the result
	{
		for(int i = n-1; i >= 0; i--)
		{
			for(int j = n-1; j > i; j--)
			{
				destInv.pdata[i*n + column] = destInv.pdata[i*n + column] - destInv.pdata[j*n + column] * destTr.pdata[i*n + j];
			}
			destInv.pdata[i*n + column] /= destTr.pdata[i*n + i];
		}
	}



	// Populate the upper-triangular matrix
	for(int i = 0; i < n; i++) // rows of dest
	{
		for(int j = 0; j < n; j++)
		{
			destTr.pdata[j*n + i] = destInv.pdata[i*n + j];
		}
	}

	for(int i =0; i< n*n; i++)
	{
		dest->pdata[i] = 0;
	}

	// Multiply [A^-1] = [L^-T][L^-1]
	int error = matrixMul(dest, &destInv, &destTr);

	return NO_ERROR + error;
};

uint8_t matrixMul(Matrix_t* dest, Matrix_t* A, Matrix_t* B)
{
	/** uint8_t error = matrixMul(&dest, &A, &B); **/
	// Returns A * B
	// If A is an M x n matrix, and
	// if B is an n x B matrix, then
	// dest must be an M x B matrix.

	// Check input dimensions.
	if(A->cols != B->rows)
		return MATRIX_INPUT_DIM_ERR;

	// Check output matrix dimensions
	if((dest->rows != A->rows) && (dest->cols != B->cols))
		return MATRIX_OUTPUT_DIM_ERR;

	// Multply
	for(int colB = 0; colB < B->cols; colB++) // Column in B
	{
		for(int rowA = 0; rowA < A->rows; rowA++) // Row A
		{
			// Element-wise multiplication and sum in this ROW of A and COL of B
			for(int i = 0; i < A->cols; i++) // Number of cols in B
			{
				dest->pdata[colB + rowA * dest->rows] += A->pdata[rowA * A->cols + i] * B->pdata[i * B->cols + colB];
			}
		}
	}
	return NO_ERROR;
};

uint8_t matrixTranspose(Matrix_t* dest, Matrix_t* A)
{
	/** uint8_t error = matrixTranspose(&dest, &A); **/
	// Returns the transpose of a square matrix A

	if(A->rows != A->cols)
		return MATRIX_INPUT_DIM_ERR;

	for(int i = 0; i < A->rows * A->rows; i++)
	{
		for(int j = 0; j < A->rows; j++)
		{
			dest->pdata[j*A->rows + i] = A->pdata[i*A->rows + j];
		}
	}

};

uint8_t matrixAdd(Matrix_t* dest, Matrix_t* A,Matrix_t* B)
{

	if(A->rows != B->rows)
		return MATRIX_INPUT_DIM_ERR;

	if(A->cols != B->cols)
		return MATRIX_INPUT_DIM_ERR;

	if(dest->rows != A->rows)
		return MATRIX_OUTPUT_DIM_ERR;

	if(dest->cols != A->cols)
		return MATRIX_OUTPUT_DIM_ERR;

	for(int i = 0; i< A->rows; i++)
	{
		for(int j = 0; j < A->cols; j++)
		{
			dest->pdata[i*A->cols + j] = A->pdata[i*A->cols + j] + B->pdata[i*A->cols + j];
		}
	}

};


#endif // OSQ_MATRIXLIB_H_INCLUDED
