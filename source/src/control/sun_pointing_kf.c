#include "control.h"
#include "math_pack.h"
#include <math.h>

void SunPointingKF(MatrixMath_s* dipoleComm, MatrixMath_s angrate_s, MatrixMath_s r_e2s,
                   Quaternion_s qEst, MatrixMath_s b_sEst, MatrixMath_s Inertia_ref,
                   ctrlgains_s* ctrlgains, MatrixMath_s angrate) {
  MatrixMath_s A_i2s = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s angrate_comm_s = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s K_error_temp = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s K_error = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s kp_temp = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s kn_diag_temp = {
    {0, 0, 0,
     0, 1, 0,
     0, 0, 1},
    3, 3
  };
  MatrixMath_s kn_temp = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s ctrltorq = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s skewMatrix = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  double P_error = 0;

  double bnorm = vectorNorm(&b_sEst);
  double angrate_comm_norm = vectorNorm(&angrate);

  q2m(qEst, &A_i2s);

  multiplyScalar(angrate_comm_norm, &A_i2s);
  multiplyMatrices(&A_i2s, &r_e2s, &angrate_comm_s);

  subtractionMatrices(&angrate_comm_s, &angrate_s, &K_error_temp);
  multiplyMatrices(&Inertia_ref, &K_error_temp, &K_error);

  P_error = Inertia_ref.arr[0] * (angrate_comm_norm - angrate_s.arr[0]);

  multiplyScalar((*ctrlgains).k, &K_error);

  multiplyScalar((*ctrlgains).kp*P_error, &kp_temp);

  multiplyScalar((*ctrlgains).kn, &kn_diag_temp);
  multiplyMatrices(&kn_diag_temp, &angrate_s, &kn_temp);

  addingMatrices(&K_error, &kp_temp, &ctrltorq);
  addingMatrices(&ctrltorq, &kn_temp, &ctrltorq);

  skew(b_sEst, &skewMatrix);
  multiplyMatrices(&skewMatrix, &ctrltorq, dipoleComm);
  multiplyScalar((1/pow(bnorm, 2)), dipoleComm);
}
