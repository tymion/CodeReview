#ifndef HEADER_MATH_PACK_H_
#define HEADER_MATH_PACK_H_

#include "structurs.h"

double factorial(double f);

float roundTest(float val);

void multiplyScalar(double scalar, MatrixMath_s* matrixA);

void multiplyQuaternionScalar(double scalar, Quaternion_s* matrixA);

void multiplyScalar6x6(double scalar, MatrixMath6x6_s* matrixA);

void multiplyScalar7x1(double scalar, MatrixMath7x1_s* matrixA);

void multiplyMatrices(MatrixMath_s* matrixA, MatrixMath_s* matrixB, MatrixMath_s* resultMatrix);

void multiplyMatrices6x6(MatrixMath6x6_s* matrixA, MatrixMath6x6_s* matrixB, MatrixMath6x6_s* resultMatrix);

void multiplyMatrices9x6and6x6(MatrixMath9x6_s* matrixA, MatrixMath6x6_s* matrixB, MatrixMath9x6_s* resultMatrix);

void multiplyMatrices6x6and9x6(MatrixMath6x6_s* matrixA, MatrixMath9x6_s* matrixB, MatrixMath9x6_s* resultMatrix);

void multiplyMatrices9x6and6x9(MatrixMath9x6_s* matrixA, MatrixMath9x6_s* matrixB, MatrixMath9x9_s* resultMatrix);

void multiplyMatrices9x6and9x9(MatrixMath9x6_s* matrixA, MatrixMath9x9_s* matrixB, MatrixMath9x6_s* resultMatrix);

void multiplyMatrixmatrixByQuaternion(MatrixMath4x4_s* matrixA, Quaternion_s* q, Quaternion_s* result);

void addingMatrices(MatrixMath_s* matrixA, MatrixMath_s* matrixB, MatrixMath_s* resultMatrix);

void addingMatrices6x6(MatrixMath6x6_s* matrixA, MatrixMath6x6_s* matrixB, MatrixMath6x6_s* resultMatrix);

void addingMatrices9x9(MatrixMath9x9_s* matrixA, MatrixMath9x9_s* matrixB, MatrixMath9x9_s* resultMatrix);

void addingMatrices7x1(MatrixMath7x1_s* matrixA, MatrixMath7x1_s* matrixB, MatrixMath7x1_s* resultMatrix);

void subtractionMatrices(MatrixMath_s* matrixA, MatrixMath_s* matrixB, MatrixMath_s* resultMatrix);

void subtractionMatrices6x6(MatrixMath_s* matrixA, MatrixMath_s* matrixB, MatrixMath_s* resultMatrix);

void transpose(MatrixMath_s* matrixA);

void transpose_al(MatrixMath_s* matrixA);

void transpose6x6(MatrixMath6x6_s* matrixA);

void transpose9x6(MatrixMath9x6_s* matrixA);

void inverse(MatrixMath_s* result);

void inverseMatrix(MatrixMath_s* result);

double vectorNorm(MatrixMath_s* matrixA);

void skew(MatrixMath_s inputMatrix, MatrixMath_s* resultMatrix);

void q2m(Quaternion_s quaternion, MatrixMath_s* result);

void q_norm(Quaternion_s* q);

void resetMatrix(MatrixMath_s* matrix);

#endif  // HEADER_MATH_PACK_H_
