/*! 
 * \file limit_angle_gr3.h
 * \brief limit an angle in ]-pi;pi]
 */

#ifndef _LIMIT_ANGLE_EX_H_
#define _LIMIT_ANGLE_EX_H_

// function prototype
void limit_angle(double *x);

void multiplyMatrices(double firstMatrix[][3], double secondMatrix[][3], double mult[][3], int rowFirst, int columnFirst, int rowSecond, int columnSecond);
void sumMatrices(double firstMatrix[][3], double secondMatrix[][3], double sum[][3], int row, int column);
void zeroMatrix(double matrix[][3], int row, int column);
void copyMatrix(double originMatrix[][3], double receivingMatrix[][3], int row, int column);
void printMatrix(double matrix[][3], int row, int column);

#endif
