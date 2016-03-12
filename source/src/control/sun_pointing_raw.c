#include "control.h"
#include "math_pack.h"
#include <math.h>

void SunPointingRaw(MatrixMath_s* dipoleComm, MatrixMath_s angrate_s, MatrixMath_s b_sEst,
                    MatrixMath_s sun_vec_meas, MatrixMath_s Inertia_ref, ctrlgains_s* ctrlgains,
                    MatrixMath_s angrate) {
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
  double angrate_comm_norm = vectorNorm(&angrate);
  double bnorm = vectorNorm(&b_sEst);

  /* angular momentum error */
  multiplyScalar(angrate_comm_norm, &sun_vec_meas);
  subtractionMatrices(&sun_vec_meas, &angrate_s, &K_error_temp);
  multiplyMatrices(&Inertia_ref, &K_error_temp, &K_error);

  /* precession error */
  P_error = Inertia_ref.arr[0] * (angrate_comm_norm - angrate_s.arr[0]);

  /* control law */
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
