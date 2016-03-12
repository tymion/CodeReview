#ifndef HEADER_DEBUG_PACK_H_
#define HEADER_DEBUG_PACK_H_

#include "structurs.h"

void displayQuaternion(Quaternion_s* A);

void displayMatrix(MatrixMath_s* A);

void displayArray(MatrixMath_s* A);

void displayMatrix7x1(MatrixMath7x1_s* A);

void displayMatrix9x1(MatrixMath9x1_s* A);

void displayMatrix6x6(MatrixMath6x6_s* A);

void displayMatrix9x9(MatrixMath9x9_s* A);

void displayMatrix4x4(MatrixMath4x4_s* A);

#endif  // HEADER_DEBUG_PACK_H_
