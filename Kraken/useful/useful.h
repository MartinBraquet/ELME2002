/*! 
 * \file limit_angle_gr3.h
 * \brief limit an angle in ]-pi;pi]
 */

#ifndef _LIMIT_ANGLE_EX_H_
#define _LIMIT_ANGLE_EX_H_

// function prototype
void limit_angle(double *x);

void multiplyMatrices(double firstMatrix[][3], double secondMatrix[][3], double mult[][3], int rowFirst, int columnFirst, int columnSecond);
void sumMatrices(double firstMatrix[][3], double secondMatrix[][3], double sum[][3], int row, int column);
void zeroMatrix(double matrix[][3], int row, int column);
void copyMatrix(double originMatrix[][3], double receivingMatrix[][3], int row, int column);
void printMatrix(double matrix[][3], int row, int column);
void inverseMatrix33(double matrix[3][3], double matrix_inv[3][3]);
void inverseMatrix22(double matrix[3][3], double matrix_inv[3][3]);
void transposeMatrix(double matrix[3][3], double matrix_T[3][3]);
void subtractMatrices(double firstMatrix[][3], double secondMatrix[][3], double sum[][3], int row, int column);
void multiplyMatrices_Vector32(double firstMatrix[3][3], double secondVector[2], double mult[2]);

#endif
