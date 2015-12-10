#ifndef HEADER_MATH_PACK_H_
#define HEADER_MATH_PACK_H_

#include "structurs.h"

double factorial(double f);

float roundTest(float val);

void multiplyScalar(double scalar, MatrixMath_s* A);

void multiplyQuaternionScalar(double scalar, Quaternion_s* A);

void multiplyScalar6x6(double scalar, MatrixMath6x6_s* A);

void multiplyScalar7x1(double scalar, MatrixMath7x1_s* A);

void multiplyMatrices(MatrixMath_s* A, MatrixMath_s* B, MatrixMath_s* resultMatrix);

void multiplyMatrices6x6(MatrixMath6x6_s* A, MatrixMath6x6_s* B, MatrixMath6x6_s* resultMatrix);

void multiplyMatrices9x6and6x6(MatrixMath9x6_s* A, MatrixMath6x6_s* B, MatrixMath9x6_s* resultMatrix);

void multiplyMatrices6x6and9x6(MatrixMath6x6_s* A, MatrixMath9x6_s* B, MatrixMath9x6_s* resultMatrix);

void multiplyMatrices9x6and6x9(MatrixMath9x6_s* A, MatrixMath9x6_s* B, MatrixMath9x9_s* resultMatrix);

void multiplyMatrices9x6and9x9(MatrixMath9x6_s* A, MatrixMath9x9_s* B, MatrixMath9x6_s* resultMatrix);

void multiplyMatrixByQuaternion(MatrixMath4x4_s* A, Quaternion_s* q, Quaternion_s* result);

void addingMatrices(MatrixMath_s* A, MatrixMath_s* B, MatrixMath_s* resultMatrix);

void addingMatrices6x6(MatrixMath6x6_s* A, MatrixMath6x6_s* B, MatrixMath6x6_s* resultMatrix);

void addingMatrices9x9(MatrixMath9x9_s* A, MatrixMath9x9_s* B, MatrixMath9x9_s* resultMatrix);

void addingMatrices7x1(MatrixMath7x1_s* A, MatrixMath7x1_s* B, MatrixMath7x1_s* resultMatrix);

void subtractionMatrices(MatrixMath_s* A, MatrixMath_s* B, MatrixMath_s* resultMatrix);

void subtractionMatrices6x6(MatrixMath_s* A, MatrixMath_s* B, MatrixMath_s* resultMatrix);

void transpose(MatrixMath_s* A);

void transpose_al(MatrixMath_s* A);

void transpose6x6(MatrixMath6x6_s* A);

void transpose9x6(MatrixMath9x6_s* A);

void inverse(MatrixMath_s* result);

void inverseMatrix(MatrixMath_s* result);

double vectorNorm(MatrixMath_s* A);

void skew(MatrixMath_s inputMatrix, MatrixMath_s* resultMatrix);

void q2m(Quaternion_s q, MatrixMath_s* result);

void q_norm(Quaternion_s* q);

void resetMatrix(MatrixMath_s* matrix);

#endif  // HEADER_MATH_PACK_H_
