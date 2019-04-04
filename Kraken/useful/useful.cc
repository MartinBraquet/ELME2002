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

void multiplyMatrices(double firstMatrix[][3], double secondMatrix[][3], double mult[][3], int rowFirst, int columnFirst, int rowSecond, int columnSecond)
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
			originMatrix[i][j] = receivingMatrix[j][i];
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
}
