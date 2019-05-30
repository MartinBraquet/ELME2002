#include "useful.h"
#include <math.h>
#include <stdio.h>

/*! \brief set an angle in the in ]-pi;pi] range
 * 
 * \param[in] x angle to limit
 * \return angle limited in ]-pi;pi]
 */
void limit_angle(double *x)
{
	while (*x <= -M_PI)
	{
		*x += 2.0*M_PI;
	}
	while (*x > M_PI)
	{
		*x -= 2.0*M_PI;
	}

	return ;
}

///////////////////////
//  multiplyMatrices
///////////////////////

void multiplyMatrices(double firstMatrix[][3], double secondMatrix[][3], double mult[][3], int rowFirst, int columnFirst, int columnSecond)
{
	int i, j, k;

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for (i = 0; i < rowFirst; ++i)
	{
		for (j = 0; j < columnSecond; ++j)
		{
			mult[i][j] = 0.0;
			for (k = 0; k < columnFirst; ++k)
			{
				mult[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
			}
		}
	}
}

///////////////////////
//  multiplyMatrices_Vector32
///////////////////////

void multiplyMatrices_Vector32(double firstMatrix[3][3], double secondVector[2], double mult[2])
{
	int j, k;

	for (j = 0; j < 3; ++j)
	{
		mult[j] = 0.0;
		for (k = 0; k < 2; ++k)
		{
			mult[j] += firstMatrix[j][k] * secondVector[k];
		}
	}
}

///////////////////////
//  sumMatrices
///////////////////////

void sumMatrices(double firstMatrix[][3], double secondMatrix[][3], double sum[][3], int row, int column)
{
	int i, j, k;

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for (i = 0; i < row; ++i)
		for (j = 0; j < column; ++j)
			sum[i][j] = firstMatrix[i][j] + secondMatrix[i][j];
}

///////////////////////
//  subtractMatrices
///////////////////////
void subtractMatrices(double firstMatrix[][3], double secondMatrix[][3], double sum[][3], int row, int column)
{
	int i, j, k;

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for (i = 0; i < row; ++i)
		for (j = 0; j < column; ++j)
			sum[i][j] = firstMatrix[i][j] - secondMatrix[i][j];
}

///////////////////////
//  zeroMatrix
///////////////////////

void zeroMatrix(double matrix[][3], int row, int column) {
	for (int i = 0; i < row; i++)
		for (int j = 0; j < column; j++)
			matrix[i][j] = 0.0;
}


///////////////////////
//  copyMatrix
///////////////////////

void copyMatrix(double originMatrix[][3], double receivingMatrix[][3], int row, int column) {
	for (int i = 0; i < row; i++)
		for (int j = 0; j < column; j++)
			receivingMatrix[i][j] = originMatrix[j][i];
}

///////////////////////
//  printMatrix
///////////////////////

void printMatrix(double matrix[][3], int row, int column) {
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++)
			printf("   %1f   ", matrix[i][j]);
		printf("\n");
	}
	printf("\n");
}

///////////////////////
//  transposeMatrix
///////////////////////

void transposeMatrix(double matrix[3][3], double matrix_T[3][3]) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			matrix_T[i][j] = matrix[j][i];
		}	
	}
}

///////////////////////
//  inverseMatrix 3x3
///////////////////////

void inverseMatrix33(double matrix[3][3], double matrix_inv[3][3]) {

    // computes the inverse of a matrix
    double det = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[2][1] * matrix[1][2]) -
                 matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
                 matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

    double invdet = 1 / det;

    matrix_inv[0][0] = (matrix[1][1] * matrix[2][2] - matrix[2][1] * matrix[1][2]) * invdet;
    matrix_inv[0][1] = (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) * invdet;
    matrix_inv[0][2] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) * invdet;
    matrix_inv[1][0] = (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) * invdet;
    matrix_inv[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) * invdet;
    matrix_inv[1][2] = (matrix[1][0] * matrix[0][2] - matrix[0][0] * matrix[1][2]) * invdet;
    matrix_inv[2][0] = (matrix[1][0] * matrix[2][1] - matrix[2][0] * matrix[1][1]) * invdet;
    matrix_inv[2][1] = (matrix[2][0] * matrix[0][1] - matrix[0][0] * matrix[2][1]) * invdet;
    matrix_inv[2][2] = (matrix[0][0] * matrix[1][1] - matrix[1][0] * matrix[0][1]) * invdet;
}

///////////////////////
//  inverseMatrix 2x2
///////////////////////

void inverseMatrix22(double matrix[3][3], double matrix_inv[3][3]) {

    // computes the inverse of a matrix
    double det = matrix[0][0] * matrix[1][1]  - matrix[0][1] * matrix[1][0];

    double invdet = 1 / det;

    matrix_inv[0][0] =   matrix[1][1] * invdet;
    matrix_inv[0][1] = - matrix[0][1] * invdet;
    matrix_inv[1][0] = - matrix[1][0] * invdet;
    matrix_inv[1][1] =   matrix[0][0] * invdet;
}
