#include <stdio.h>
#include <stdlib.h>
#include "math_pack.h"
#include <gtest/gtest.h>

GTEST_TEST(MultiplyMatrixWithScalar, PositiveScalar)
{
    int scalar = 5;
    MatrixMath_s matrix = {
        .arr = { 1, 2, 3,
                 2, 3, 4,
                 4, 5, 6 },
        .row = 3,
        .col = 3
    };
    MatrixMath_s expectedMatrix = {
        .arr = { 5, 10, 15,
                 10, 15, 20,
                 20, 25, 30 },
        .row = 3,
        .col = 3
    };
    multiplyScalar(scalar, &matrix);
    for (int i = 0; i < 9; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], matrix.arr[i]);
    }
}

GTEST_TEST(MultiplyMatrixWithScalar, NegativeScalar)
{
    int scalar = -5;
    MatrixMath_s matrix = {
        .arr = { 1, 2, 3,
                 2, 3, 4,
                 4, 5, 6 },
        .row = 3,
        .col = 3
    };
    MatrixMath_s expectedMatrix = {
        .arr = { -5, -10, -15,
                 -10, -15, -20,
                 -20, -25, -30 },
        .row = 3,
        .col = 3
    };
    multiplyScalar(scalar, &matrix);
    for (int i = 0; i < 9; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], matrix.arr[i]);
    }
}

GTEST_TEST(MultiplyMatrixWithScalar, ZeroScalar)
{
    int scalar = 0;
    MatrixMath_s matrix = {
        .arr = { 1, 2, 3,
                 2, 3, 4,
                 4, 5, 6 },
        .row = 3,
        .col = 3
    };
    MatrixMath_s expectedMatrix = {
        .arr = { 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0 },
        .row = 3,
        .col = 3
    };
    multiplyScalar(scalar, &matrix);
    for (int i = 0; i < 9; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], matrix.arr[i]);
    }
}

GTEST_TEST(MultiplyQuaternionWithScalar, NegativeScalar)
{
    int scalar = -5;
    Quaternion_s matrix = {
        .arr = { 1, 2, 3, 4 },
    };
    Quaternion_s expectedMatrix = {
        .arr = { -5, -10, -15, -20 },
    };
    multiplyQuaternionScalar(scalar, &matrix);
    for (int i = 0; i < 4; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], matrix.arr[i]);
    }
}

GTEST_TEST(MultiplyQuaternionWithScalar, PositiveScalar)
{
    int scalar = 5;
    Quaternion_s matrix = {
        .arr = { 1, 2, 3, 4 },
    };
    Quaternion_s expectedMatrix = {
        .arr = { 5, 10, 15, 20 },
    };
    multiplyQuaternionScalar(scalar, &matrix);
    for (int i = 0; i < 4; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], matrix.arr[i]);
    }
}

GTEST_TEST(MultiplyQuaternionWithScalar, ZeroScalar)
{
    int scalar = 0;
    Quaternion_s matrix = {
        .arr = { 1, 2, 3, 4 },
    };
    Quaternion_s expectedMatrix = {
        .arr = { 0, 0, 0, 0 },
    };
    multiplyQuaternionScalar(scalar, &matrix);
    for (int i = 0; i < 4; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], matrix.arr[i]);
    }
}

GTEST_TEST(MultiplyMatrix6x6WithScalar, NegativeScalar)
{
    int scalar = -5;
    MatrixMath6x6_s matrix = {
        .arr = { 1, 2, 3, 4, 5, 6,
                 2, 3, 4, 5, 6, 7,
                 3, 4, 5, 6, 7, 8,
                 4, 5, 6, 7, 8, 9,
                 5, 6, 7, 8, 9, 10,
                 6, 7, 8, 9, 10, 11 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s expectedMatrix = {
        .arr = { -5, -10, -15, -20, -25, -30,
                 -10, -15, -20, -25, -30, -35,
                 -15, -20, -25, -30, -35, -40,
                 -20, -25, -30, -35, -40, -45,
                 -25, -30, -35, -40, -45, -50,
                 -30, -35, -40, -45, -50, -55 },
        .row = 6,
        .col = 6
    };
    multiplyScalar6x6(scalar, &matrix);
    for (int i = 0; i < 36; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], matrix.arr[i]);
    }
}

GTEST_TEST(MultiplyMatrix6x6WithScalar, PositiveScalar)
{
    int scalar = 5;
    MatrixMath6x6_s matrix = {
        .arr = { 1, 2, 3, 4, 5, 6,
                 2, 3, 4, 5, 6, 7,
                 3, 4, 5, 6, 7, 8,
                 4, 5, 6, 7, 8, 9,
                 5, 6, 7, 8, 9, 10,
                 6, 7, 8, 9, 10, 11 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s expectedMatrix = {
        .arr = { 5, 10, 15, 20, 25, 30,
                 10, 15, 20, 25, 30, 35,
                 15, 20, 25, 30, 35, 40,
                 20, 25, 30, 35, 40, 45,
                 25, 30, 35, 40, 45, 50,
                 30, 35, 40, 45, 50, 55 },
        .row = 6,
        .col = 6
    };
    multiplyScalar6x6(scalar, &matrix);
    for (int i = 0; i < 36; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], matrix.arr[i]);
    }
}

GTEST_TEST(MultiplyMatrix6x6WithScalar, ZeroScalar)
{
    int scalar = 0;
    MatrixMath6x6_s matrix = {
        .arr = { 1, 2, 3, 4, 5, 6,
                 2, 3, 4, 5, 6, 7,
                 3, 4, 5, 6, 7, 8,
                 4, 5, 6, 7, 8, 9,
                 5, 6, 7, 8, 9, 10,
                 6, 7, 8, 9, 10, 11 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s expectedMatrix = {
        .arr = { 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0 },
        .row = 6,
        .col = 6
    };
    multiplyScalar6x6(scalar, &matrix);
    for (int i = 0; i < 36; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], matrix.arr[i]);
    }
}

GTEST_TEST(MultiplyMatrix7x1WithScalar, PositiveScalar)
{
    int scalar = 5;
    MatrixMath7x1_s matrix = {
        .arr = { 1, 2, 3, 2, 3, 4, 5 }
    };
    MatrixMath7x1_s expectedMatrix = {
        .arr = { 5, 10, 15, 10, 15, 20, 25 }
    };
    multiplyScalar7x1(scalar, &matrix);
    for (int i = 0; i < 7; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], matrix.arr[i]);
    }
}

GTEST_TEST(MultiplyMatrix7x1WithScalar, NegativeScalar)
{
    int scalar = -5;
    MatrixMath7x1_s matrix = {
        .arr = { 1, 2, 3, 2, 3, 4, 5 }
    };
    MatrixMath7x1_s expectedMatrix = {
        .arr = { -5, -10, -15, -10, -15, -20, -25 }
    };
    multiplyScalar7x1(scalar, &matrix);
    for (int i = 0; i < 7; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], matrix.arr[i]);
    }
}

GTEST_TEST(MultiplyMatrix7x1WithScalar, ZeroScalar)
{
    int scalar = 0;
    MatrixMath7x1_s matrix = {
        .arr = { 1, 2, 3, 2, 3, 4, 5 }
    };
    MatrixMath7x1_s expectedMatrix = {
        .arr = { 0, 0, 0, 0, 0, 0, 0 }
    };
    multiplyScalar7x1(scalar, &matrix);
    for (int i = 0; i < 7; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], matrix.arr[i]);
    }
}

GTEST_TEST(MultiplyMatrices, PositiveScalar)
{
    MatrixMath_s matrixA = {
        .arr = { 1, 2, 3,
                 2, 3, 4,
                 4, 5, 6 },
        .row = 3,
        .col = 3
    };
    MatrixMath_s matrixB = {
        .arr = { 1, 2, 3,
                 2, 3, 4,
                 4, 5, 6 },
        .row = 3,
        .col = 3
    };
    MatrixMath_s resultMatrix = {
        .arr = { 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0 },
        .row = 3,
        .col = 3
    };
    MatrixMath_s expectedMatrix = {
        .arr = { 17, 23, 29,
                 24, 33, 42,
                 38, 53, 68 },
        .row = 3,
        .col = 3
    };
    multiplyMatrices(&matrixA, &matrixB, &resultMatrix);
    for (int i = 0; i < 9; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], resultMatrix.arr[i]);
    }
}
GTEST_TEST(MultiplyMatrices6x6, PositiveValue)
{
    MatrixMath6x6_s matrixA = {
        .arr = { 1, 2, 3, 4, 5, 6,
                 2, 3, 4, 5, 6, 7,
                 3, 4, 5, 6, 7, 8,
                 4, 5, 6, 7, 8, 9,
                 5, 6, 7, 8, 9, 10,
                 6, 7, 8, 9, 10, 11 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s matrixB = {
        .arr = { 1, 2, 3, 4, 5, 6,
                 2, 3, 4, 5, 6, 7,
                 3, 4, 5, 6, 7, 8,
                 4, 5, 6, 7, 8, 9,
                 5, 6, 7, 8, 9, 10,
                 6, 7, 8, 9, 10, 11 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s resultMatrix = {
        .arr = { 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s expectedMatrix = {
        .arr = { 91, 112, 133, 154, 175, 196,
                 112, 139, 166, 193, 220, 247,
                 133, 166, 199, 232, 265, 298,
                 154, 193, 232, 271, 310, 349,
                 175, 220, 265, 310, 355, 400,
                 196, 247, 298, 349, 400, 451 },
        .row = 6,
        .col = 6
    };
    multiplyMatrices6x6(&matrixA, &matrixB, &resultMatrix);
    for (int i = 0; i < 36; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], resultMatrix.arr[i]);
    }
}

GTEST_TEST(MultiplyMatrices6x6, NegativeValue)
{
    MatrixMath6x6_s matrixA = {
        .arr = { 1, 2, 3, 4, 5, 6,
                 2, 3, 4, 5, 6, 7,
                 3, 4, 5, 6, 7, 8,
                 4, 5, 6, 7, 8, 9,
                 5, 6, 7, 8, 9, 10,
                 6, 7, 8, 9, 10, 11 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s matrixB = {
        .arr = { -1, -2, -3, -4, -5, -6,
                 -2, -3, -4, -5, -6, -7,
                 -3, -4, -5, -6, -7, -8,
                 -4, -5, -6, -7, -8, -9,
                 -5, -6, -7, -8, -9, -10,
                 -6, -7, -8, -9, -10, -11 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s resultMatrix = {
        .arr = { 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s expectedMatrix = {
        .arr = { -91, -112, -133, -154, -175, -196,
                 -112, -139, -166, -193, -220, -247,
                 -133, -166, -199, -232, -265, -298,
                 -154, -193, -232, -271, -310, -349,
                 -175, -220, -265, -310, -355, -400,
                 -196, -247, -298, -349, -400, -451 },
        .row = 6,
        .col = 6
    };
    multiplyMatrices6x6(&matrixA, &matrixB, &resultMatrix);
    for (int i = 0; i < 36; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], resultMatrix.arr[i]);
    }
}

GTEST_TEST(MyTest, ThisOneWorks)
{
/*    double result;
    printf("Function: AssLegendre\n");
    result = AssLegendre(5, 2, 0.5);
    printf("Result: %f\n", result);
    */
	EXPECT_EQ(2, 2);
}

GTEST_TEST(AddingMatrices, PositiveValues)
{
    MatrixMath_s matrixA = {
        .arr = { 1, 2, 3,
                 2, 3, 4,
                 3, 4, 5 },
        .row = 3,
        .col = 3
    };
    MatrixMath_s matrixB = {
        .arr = { 1, 2, 3,
                 2, 3, 4,
                 3, 4, 5 },
        .row = 3,
        .col = 3
    };
    MatrixMath_s resultMatrix = {
        .arr = { 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0 },
        .row = 3,
        .col = 3
    };
    MatrixMath_s expectedMatrix = {
        .arr = { 2, 4, 6,
                 4, 6, 8,
                 6, 8, 10 },
        .row = 3,
        .col = 3
    };
    addingMatrices(&matrixA, &matrixB, &resultMatrix);
    for (int i = 0; i < 9; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], resultMatrix.arr[i]);
    }
}

GTEST_TEST(AddingMatrices6x6, PositiveValues)
{
    MatrixMath6x6_s matrixA = {
        .arr = { 1, 2, 3, 4, 5, 6,
                 2, 3, 4, 5, 6, 7,
                 3, 4, 5, 6, 7, 8,
                 4, 5, 6, 7, 8, 9,
                 5, 6, 7, 8, 9, 10,
                 6, 7, 8, 9, 10, 11 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s matrixB = {
        .arr = { 1, 2, 3, 4, 5, 6,
                 2, 3, 4, 5, 6, 7,
                 3, 4, 5, 6, 7, 8,
                 4, 5, 6, 7, 8, 9,
                 5, 6, 7, 8, 9, 10,
                 6, 7, 8, 9, 10, 11 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s resultMatrix = {
        .arr = { 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s expectedMatrix = {
        .arr = { 2, 4, 6, 8, 10, 12,
                 4, 6, 8, 10, 12, 14,
                 6, 8, 10, 12, 14, 16,
                 8, 10, 12, 14, 16, 18,
                 10, 12, 14, 16, 18, 20,
                 12, 14, 16, 18, 20, 22 },
        .row = 6,
        .col = 6
    };
    addingMatrices6x6(&matrixA, &matrixB, &resultMatrix);
    for (int i = 0; i < 36; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], resultMatrix.arr[i]);
    }
}

GTEST_TEST(AddingMatrices6x6, NegativeValues)
{
    MatrixMath6x6_s matrixA = {
        .arr = { 1, 2, 3, 4, 5, 6,
                 2, 3, 4, 5, 6, 7,
                 3, 4, 5, 6, 7, 8,
                 4, 5, 6, 7, 8, 9,
                 5, 6, 7, 8, 9, 10,
                 6, 7, 8, 9, 10, 11 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s matrixB = {
        .arr = { -1, -2, -3, -4, -5, -6,
                 -2, -3, -4, -5, -6, -7,
                 -3, -4, -5, -6, -7, -8,
                 -4, -5, -6, -7, -8, -9,
                 -5, -6, -7, -8, -9, -10,
                 -6, -7, -8, -9, -10, -11 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s resultMatrix = {
        .arr = { -1, -1, -1, -1, -1, -1,
                 -1, -1, -1, -1, -1, -1,
                 -1, -1, -1, -1, -1, -1,
                 -1, -1, -1, -1, -1, -1,
                 -1, -1, -1, -1, -1, -1,
                 -1, -1, -1, -1, -1, -1 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s expectedMatrix = {
        .arr = { 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0 },
        .row = 6,
        .col = 6
    };
    addingMatrices6x6(&matrixA, &matrixB, &resultMatrix);
    for (int i = 0; i < 36; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], resultMatrix.arr[i]);
    }
}

GTEST_TEST(AddingMatrices9x9, NegativeValues)
{
    MatrixMath9x9_s matrixA = {
        .arr = { 1, 2, 3, 4, 5, 6, 7, 8, 9,
                 2, 3, 4, 5, 6, 7, 8, 9, 10,
                 3, 4, 5, 6, 7, 8, 9, 10, 11,
                 4, 5, 6, 7, 8, 9, 10, 11, 12,
                 5, 6, 7, 8, 9, 10, 11, 12, 13,
                 6, 7, 8, 9, 10, 11 12, 13, 14,
                 7, 8, 9, 10, 11 12, 13, 14, 15,
                 8, 9, 10, 11 12, 13, 14, 15, 16,
                 9, 10, 11 12, 13, 14, 15, 16, 17 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s matrixB = {
        .arr = { -1, -2, -3, -4, -5, -6,
                 -2, -3, -4, -5, -6, -7,
                 -3, -4, -5, -6, -7, -8,
                 -4, -5, -6, -7, -8, -9,
                 -5, -6, -7, -8, -9, -10,
                 -6, -7, -8, -9, -10, -11 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s resultMatrix = {
        .arr = { -1, -1, -1, -1, -1, -1,
                 -1, -1, -1, -1, -1, -1,
                 -1, -1, -1, -1, -1, -1,
                 -1, -1, -1, -1, -1, -1,
                 -1, -1, -1, -1, -1, -1,
                 -1, -1, -1, -1, -1, -1 },
        .row = 6,
        .col = 6
    };
    MatrixMath6x6_s expectedMatrix = {
        .arr = { 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0 },
        .row = 6,
        .col = 6
    };
    addingMatrices6x6(&matrixA, &matrixB, &resultMatrix);
    for (int i = 0; i < 36; i++) {
    	EXPECT_EQ(expectedMatrix.arr[i], resultMatrix.arr[i]);
    }
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
