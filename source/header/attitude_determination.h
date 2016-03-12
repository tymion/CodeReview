#ifndef HEADER_ATTITUDE_DETERMINATION_H_
#define HEADER_ATTITUDE_DETERMINATION_H_

#include "structurs.h"

void PropagateState(MatrixMath7x1_s* result, MatrixMath7x1_s z, MatrixMath_s ctrlTorque_prev,
                    MatrixMath_s Inertia_ref);

void WahbaTriadWeight(MatrixMath_s* result, MatrixMath_s b1, MatrixMath_s b2, MatrixMath_s r1, MatrixMath_s r2,
                                   double sigmaWahba_b1, double sigmaWahba_b2, double sigmaWahba_r1, double sigmaWahba_r2);

void EkfSunVector(MatrixMath7x1_s* result7x1, MatrixMath6x6_s* result6x6, MatrixMath9x1_s* result9x1, MatrixMath9x9_s* result9x9,
                  MatrixMath7x1_s X_k, MatrixMath6x6_s P_k, MatrixMath9x1_s Z, double eclipse, MatrixMath_s b_eci, MatrixMath_s r_e2s_eci,
                  double time_step, MatrixMath9x9_s KalmanCovariance_R, MatrixMath6x6_s KalmanCovariance_Q, MatrixMath_s Inertia_ref,
                  MatrixMath_s ctrlTorque_prev);

#endif  // HEADER_ATTITUDE_DETERMINATION_H_
