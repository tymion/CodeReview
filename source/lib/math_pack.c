#include "math_pack.h"

#define M_PI 3.14159265358979323846

double factorial(double f) {
  if (f == 0)
    return 1;
  return(f * factorial(f - 1));
}

float roundTest(float val) {
  return val;
}

void multiplyScalar(double scalar, MatrixMath_s* matrixA) {
  int i, j;
  for (i = 0; i <3; i++) {
    for (j = 0; j < 3; j++) {
      (*matrixA).arr[i * 3 + j]  = (*matrixA).arr[i * 3 + j] * scalar;
    }
  }
}

void multiplyQuaternionScalar(double scalar, Quaternion_s* matrixA) {
  int i;
  for (i = 0; i <4; i++) {
    (*matrixA).arr[i] = (*matrixA).arr[i] * scalar;
  }
}

void multiplyScalar6x6(double scalar, MatrixMath6x6_s* matrixA) {
  int i, j;
  for (i = 0; i <6; i++) {
    for (j = 0; j < 6; j++) {
      (*matrixA).arr[i * 6 + j]  = (*matrixA).arr[i * 6 + j] * scalar;
    }
  }
}

void multiplyScalar7x1(double scalar, MatrixMath7x1_s* matrixA) {
  int i;
  for (i = 0; i <7; i++) {
    (*matrixA).arr[i]  = (*matrixA).arr[i] * scalar;
  }
}

void multiplyMatrices(MatrixMath_s* matrixA, MatrixMath_s* matrixB, MatrixMath_s* resultMatrix) {
  int i, j, k;
  if ((*matrixA).col == (*matrixB).row) {
    for (i = 0; i <3; i++) {
      for (j = 0; j < 3; j++) {
        (*resultMatrix).arr[i * 3 + j]  = 0.0;
        for (k = 0; k < 3; k++)
          (*resultMatrix).arr[i * 3 + j]  += (*matrixA).arr[i * 3 + k] * (*matrixB).arr[k * 3 + j];
      }
    }
  }
}

void multiplyMatrices6x6(MatrixMath6x6_s* matrixA, MatrixMath6x6_s* matrixB, MatrixMath6x6_s* resultMatrix) {
  int i, j, k;
  if ((*matrixA).col == (*matrixB).row) {
    for (i = 0; i <6; i++) {
      for (j = 0; j < 6; j++) {
        (*resultMatrix).arr[i * 6 + j]  = 0.0;
        for (k = 0; k < 6; k++)
          (*resultMatrix).arr[i * 6 + j]  += (*matrixA).arr[i * 6 + k] * (*matrixB).arr[k * 6 + j];
      }
    }
  }
}

void multiplyMatrices9x6and6x6(MatrixMath9x6_s* matrixA, MatrixMath6x6_s* matrixB, MatrixMath9x6_s* resultMatrix) {
  int i, j, k;
  if ((*matrixA).col == (*matrixB).row) {
    for (i = 0; i <9; i++) {
      for (j = 0; j < 6; j++) {
        (*resultMatrix).arr[i * 9 + j]  = 0.0;
        for (k = 0; k < 6; k++)
          (*resultMatrix).arr[i * 9 + j]  += (*matrixA).arr[i * 9 + k] * (*matrixB).arr[k * 6 + j];
      }
    }
  }
}

void multiplyMatrices6x6and9x6(MatrixMath6x6_s* matrixA, MatrixMath9x6_s* matrixB, MatrixMath9x6_s* resultMatrix) {
  int i, j, k;
  if ((*matrixA).col == (*matrixB).row) {
    for (i = 0; i <6; i++) {
      for (j = 0; j < 9; j++) {
        (*resultMatrix).arr[i * 6 + j]  = 0.0;
        for (k = 0; k < 6; k++)
          (*resultMatrix).arr[i * 9 + j]  += (*matrixA).arr[i * 9 + k] * (*matrixB).arr[k * 6 + j];
      }
    }
  }
}

void multiplyMatrices9x6and6x9(MatrixMath9x6_s* matrixA, MatrixMath9x6_s* matrixB, MatrixMath9x9_s* resultMatrix) {
  int i, j, k;
  if ((*matrixA).col == (*matrixB).row) {
    for (i = 0; i <9; i++) {
      for (j = 0; j < 6; j++) {
        (*resultMatrix).arr[i * 9 + j]  = 0.0;
        for (k = 0; k < 9; k++)
          (*resultMatrix).arr[i * 9 + j]  += (*matrixA).arr[i * 9 + k] * (*matrixB).arr[k * 9 + j];
      }
    }
  }
}

void multiplyMatrices9x6and9x9(MatrixMath9x6_s* matrixA, MatrixMath9x9_s* matrixB, MatrixMath9x6_s* resultMatrix) {
  int i, j, k;
  if ((*matrixA).col == (*matrixB).row) {
    for (i = 0; i <6; i++) {
      for (j = 0; j < 9; j++) {
        (*resultMatrix).arr[i * 6 + j]  = 0.0;
        for (k = 0; k < 6; k++)
          (*resultMatrix).arr[i * 9 + j]  += (*matrixA).arr[i * 9 + k] * (*matrixB).arr[k * 6 + j];
      }
    }
  }
}

void multiplyMatrixmatrixByQuaternion(MatrixMath4x4_s* matrixA, Quaternion_s* q, Quaternion_s* result) {
  int i, j;
  for (i = 0; i < 4; i++) {
    (*result).arr[i]  = 0.0;
    for (j = 0; j < 4; j++) {
      (*result).arr[i] += (*matrixA).arr[i * 4 + j] * (*q).arr[j];
    }
  }
}

void addingMatrices(MatrixMath_s* matrixA, MatrixMath_s* matrixB, MatrixMath_s* resultMatrix) {
  int i, j;
  for (i = 0; i <3; i++) {
    for (j = 0; j < 3; j++) {
      (*resultMatrix).arr[i * 3 + j]  = (*matrixA).arr[i * 3 + j] + (*matrixB).arr[i * 3 + j];
    }
  }
}

void addingMatrices6x6(MatrixMath6x6_s* matrixA, MatrixMath6x6_s* matrixB, MatrixMath6x6_s* resultMatrix) {
  int i, j;
  for (i = 0; i <6; i++) {
    for (j = 0; j < 6; j++) {
      (*resultMatrix).arr[i * 6 + j]  = (*matrixA).arr[i * 6 + j] + (*matrixB).arr[i * 6 + j];
    }
  }
}

void addingMatrices9x9(MatrixMath9x9_s* matrixA, MatrixMath9x9_s* matrixB, MatrixMath9x9_s* resultMatrix) {
  int i, j;
  for (i = 0; i <9; i++) {
    for (j = 0; j < 9; j++) {
      (*resultMatrix).arr[i * 9 + j]  = (*matrixA).arr[i * 9 + j] + (*matrixB).arr[i * 9 + j];
    }
  }
}

void addingMatrices7x1(MatrixMath7x1_s* matrixA, MatrixMath7x1_s* matrixB, MatrixMath7x1_s* resultMatrix) {
  int i;
  for (i = 0; i <7; i++) {
    (*resultMatrix).arr[i]  = (*matrixA).arr[i] + (*matrixB).arr[i];
  }
}

void subtractionMatrices(MatrixMath_s* matrixA, MatrixMath_s* matrixB, MatrixMath_s* resultMatrix) {
  int i, j;
  for (i = 0; i <3; i++) {
    for (j = 0; j < 3; j++) {
      (*resultMatrix).arr[i * 3 + j]  = (*matrixA).arr[i * 3 + j] - (*matrixB).arr[i * 3 + j];
    }
  }
}

void subtractionMatrices6x6(MatrixMath_s* matrixA, MatrixMath_s* matrixB, MatrixMath_s* resultMatrix) {
  int i, j;
  for (i = 0; i <6; i++) {
    for (j = 0; j < 6; j++) {
      (*resultMatrix).arr[i * 6 + j]  = (*matrixA).arr[i * 6 + j] - (*matrixB).arr[i * 6 + j];
    }
  }
}

void transpose(MatrixMath_s* matrixA) {
  MatrixMath_s tmp = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
     3, 3
  };
  int i, k;
  for (i = 0; i < 3; i++) {
    for (k = 0; k < 3; k++) {
      tmp.arr[i * 3 + k] = (*matrixA).arr[k * 3 + i];
    }
  }
  tmp.col = (*matrixA).row;
  tmp.row = (*matrixA).col;
  (*matrixA) = tmp;
}

void transpose_al(MatrixMath_s* matrixA) {
  int w = (*matrixA).row;
  int h = (*matrixA).col;
  int start, next, i;
  double tmp;

  for (start = 0; start <= w * h - 1; start++) {
    next = start;
    i = 0;
    do {  i++;
      next = (next % h) * w + next / h;
    } while (next > start);
    if (next < start || i == 1) continue;

    tmp = (*matrixA).arr[next = start];
    do {
      i = (next % h) * w + next / h;
      (*matrixA).arr[next] = (i == start) ? tmp : (*matrixA).arr[i];
      next = i;
    } while (next > start);
  }
}

void transpose6x6(MatrixMath6x6_s* matrixA) {
  MatrixMath6x6_s tmp = {
    {0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0},
     6, 6
  };
  int i, k;
  for (i = 0; i < 6; i++) {
    for (k = 0; k < 6; k++) {
      tmp.arr[i * 6 + k] = (*matrixA).arr[k * 6 + i];
    }
  }
  tmp.col = (*matrixA).row;
  tmp.row = (*matrixA).col;
  (*matrixA) = tmp;
}

void transpose9x6(MatrixMath9x6_s* matrixA) {
  MatrixMath9x6_s tmp = {
    {0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0},
     9, 6
  };
  int i, k;
  if ((*matrixA).row == 6) {
    tmp.col = 6;
    tmp.row = 9;
    for (i = 0; i < 9; i++) {
      for (k = 0; k < 6; k++) {
        tmp.arr[i * 9 + k] = (*matrixA).arr[k * 9 + i];
      }
    }
  } else {
    tmp.col = 9;
    tmp.row = 6;
    for (i = 0; i < 6; i++) {
      for (k = 0; k < 9; k++) {
        tmp.arr[i * 6 + k] = (*matrixA).arr[k * 6 + i];
      }
    }
  }
  (*matrixA) = tmp;
}

void inverse(MatrixMath_s* result) {
  double determinant = 0;
  double a11, a12, a13, a21, a22, a23, a31, a32, a33;
  a11 = (*result).arr[0];
  a12 = (*result).arr[1];
  a13 = (*result).arr[2];
  a21 = (*result).arr[3];
  a22 = (*result).arr[4];
  a23 = (*result).arr[5];
  a31 = (*result).arr[6];
  a32 = (*result).arr[7];
  a33 = (*result).arr[8];

  (*result).arr[0] = a22*a33 - a23*a32;
  (*result).arr[1] = a13*a32 - a12*a33;
  (*result).arr[2] = a12*a23 - a13*a22;
  (*result).arr[3] = a23*a31 - a21*a33;
  (*result).arr[4] = a11*a33 - a13*a31;
  (*result).arr[5] = a13*a21 - a11*a23;
  (*result).arr[6] = a21*a32 - a22*a31;
  (*result).arr[7] = a12*a31 - a11*a32;
  (*result).arr[8] = a11*a22 - a12*a21;

  determinant = a11*a22*a33+a21*a32*a13+a31*a12*a23-a11*a32*a23-a31*a22*a13-a21*a12*a33;
  multiplyScalar(1/determinant, result);
}

void inverseMatrix(MatrixMath_s* result) {
  double determinant = 0;
  double a11, a12, a13, a21, a22, a23, a31, a32, a33;
  a11 = (*result).arr[0];
  a12 = (*result).arr[1];
  a13 = (*result).arr[2];
  a21 = (*result).arr[3];
  a22 = (*result).arr[4];
  a23 = (*result).arr[5];
  a31 = (*result).arr[6];
  a32 = (*result).arr[7];
  a33 = (*result).arr[8];

  (*result).arr[0] = a22*a33 - a23*a32;
  (*result).arr[1] = a13*a32 - a12*a33;
  (*result).arr[2] = a12*a23 - a13*a22;
  (*result).arr[3] = a23*a31 - a21*a33;
  (*result).arr[4] = a11*a33 - a13*a31;
  (*result).arr[5] = a13*a21 - a11*a23;
  (*result).arr[6] = a21*a32 - a22*a31;
  (*result).arr[7] = a12*a31 - a11*a32;
  (*result).arr[8] = a11*a22 - a12*a21;

  determinant = a11*a22*a33+a21*a32*a13+a31*a12*a23-a11*a32*a23-a31*a22*a13-a21*a12*a33;
  multiplyScalar(1/determinant, result);
}

double vectorNorm(MatrixMath_s* matrixA) {
  double sum;
  double sum_i;
  int i, k;

  sum = 0;
  for (i = 0; i < (*matrixA).row; i++) {
    for (k = 0; k < (*matrixA).col; k++) {
      sum_i = (*matrixA).arr[i*3+k] * (*matrixA).arr[i*3+k];
      sum += sum_i;
    }
  }
  sum = sqrt(sum);
  return sum;
}

void skew(MatrixMath_s inputMatrix, MatrixMath_s* resultMatrix) {
  (*resultMatrix).arr[0] = 0.0;
  (*resultMatrix).arr[1] = -1*inputMatrix.arr[6];
  (*resultMatrix).arr[2] = inputMatrix.arr[3];
  (*resultMatrix).arr[3] = inputMatrix.arr[6];
  (*resultMatrix).arr[4] = 0.0;
  (*resultMatrix).arr[5] = -1*inputMatrix.arr[0];
  (*resultMatrix).arr[6] = -1*inputMatrix.arr[3];
  (*resultMatrix).arr[7] = inputMatrix.arr[0];
  (*resultMatrix).arr[8] = 0.0;
}

void q2m(Quaternion_s q, MatrixMath_s* result) {
  MatrixMath_s temp_result1 = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
     3, 3
  };
  MatrixMath_s temp_1 = {
    {1, 0, 0,
     0, 1, 0,
     0, 0, 1},
    3, 3
  };
  MatrixMath_s temp_2 = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
     3, 3
  };
  MatrixMath_s temp_3 = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
     3, 3
  };
  MatrixMath_s q_1_3 = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
     3, 1
  };
  MatrixMath_s q_1_3_trans = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
     1, 3
  };
  double temp_var_1 = 0;
  double temp_var_2 = 0;
  q_1_3.arr[0] = q.arr[0];
  q_1_3.arr[3] = q.arr[1];
  q_1_3.arr[6] = q.arr[2];

  q_1_3_trans.arr[0] = q.arr[0];
  q_1_3_trans.arr[1] = q.arr[1];
  q_1_3_trans.arr[2] = q.arr[3];

  temp_var_1 = pow(q.arr[3], 2) - pow(vectorNorm(&q_1_3), 2);
  temp_var_2 = 2 * q.arr[3];

  multiplyScalar(temp_var_1, &temp_1);

  skew(q_1_3, &temp_2);
  multiplyScalar(temp_var_2, &temp_2);

  multiplyMatrices(&q_1_3, &q_1_3_trans, &temp_3);
  multiplyScalar(2, &temp_3);
  subtractionMatrices(&temp_1, &temp_2, &temp_result1);
  addingMatrices(&temp_result1, &temp_3, result);
}

void q_norm(Quaternion_s* q) {
  double norm  = sqrt(pow((*q).arr[0], 2) + pow((*q).arr[1], 2)
  + pow((*q).arr[2], 2) + pow((*q).arr[3], 2));
  multiplyQuaternionScalar(1/norm, q);
}

void resetMatrix(MatrixMath_s* matrix) {
  int i, j;
  for (i = 0; i <3; i++) {
    for (j = 0; j < 3; j++) {
      (*matrix).arr[i * 3 + j]  = 0.0;
    }
  }
}
