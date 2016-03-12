#include "attitude_determination.h"
#include "math_pack.h"

void PropagateState(MatrixMath7x1_s* result, MatrixMath7x1_s z, MatrixMath_s ctrlTorque_prev, MatrixMath_s Inertia_ref ) {
  MatrixMath_s angrate = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s deriv_angrate = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  Quaternion_s q = {
    {0, 0, 0, 0}
  };
  Quaternion_s deriv_q = {
    {0, 0, 0, 0}
  };
  MatrixMath4x4_s tmp_deriv = {
    {0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 0},
     4, 4
  };
  MatrixMath_s tmp_a = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s tmp_b = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s tmp_c = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };

  q.arr[0] = z.arr[0];
  q.arr[1] = z.arr[1];
  q.arr[2] = z.arr[2];
  q.arr[3] = z.arr[3];

  angrate.arr[0] = z.arr[4];
  angrate.arr[3] = z.arr[5];
  angrate.arr[6] = z.arr[6];

  q_norm(&q);
  skew(angrate, &tmp_a);
  multiplyScalar(-1, &tmp_a);

  tmp_b = angrate;
  multiplyScalar(-1, &tmp_b);

  tmp_deriv.arr[0] = tmp_a.arr[0];
  tmp_deriv.arr[1] = tmp_a.arr[1];
  tmp_deriv.arr[2] = tmp_a.arr[2];
  tmp_deriv.arr[4] = tmp_a.arr[3];
  tmp_deriv.arr[5] = tmp_a.arr[4];
  tmp_deriv.arr[6] = tmp_a.arr[5];
  tmp_deriv.arr[8] = tmp_a.arr[6];
  tmp_deriv.arr[9] = tmp_a.arr[7];
  tmp_deriv.arr[10] = tmp_a.arr[8];

  tmp_deriv.arr[3] = angrate.arr[0];
  tmp_deriv.arr[7] = angrate.arr[3];
  tmp_deriv.arr[11] = angrate.arr[6];

  tmp_deriv.arr[12] = tmp_b.arr[0];
  tmp_deriv.arr[13] = tmp_b.arr[3];
  tmp_deriv.arr[14] = tmp_b.arr[6];
  tmp_deriv.arr[15] = 0;
  multiplyQuaternionScalar(0.5, &q);
  multiplyMatrixByQuaternion(&tmp_deriv, &q, &deriv_q);

  resetMatrix(&tmp_a);
  skew(angrate, &tmp_a);
  resetMatrix(&tmp_b);
  multiplyMatrices(&Inertia_ref, &angrate, &tmp_b);
  resetMatrix(&tmp_c);
  multiplyMatrices(&tmp_a, &tmp_b, &tmp_c);
  resetMatrix(&tmp_a);
  subtractionMatrices(&ctrlTorque_prev, &tmp_c, &tmp_a);
  inverse(&Inertia_ref);
  multiplyMatrices(&Inertia_ref, &tmp_a, &deriv_angrate);

  (*result).arr[0] = deriv_q.arr[0];
  (*result).arr[1] = deriv_q.arr[1];
  (*result).arr[2] = deriv_q.arr[2];
  (*result).arr[3] = deriv_q.arr[3];
  (*result).arr[4] = deriv_angrate.arr[0];
  (*result).arr[5] = deriv_angrate.arr[3];
  (*result).arr[6] = deriv_angrate.arr[6];
}
