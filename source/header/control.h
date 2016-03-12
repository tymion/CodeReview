#ifndef HEADER_CONTROL_H_
#define HEADER_CONTROL_H_

#include "structurs.h"

void Detumbling(MatrixMath_s* commDipole, MatrixMath_s bMeas, MatrixMath_s bDeriv, double ctrlgains_bdot);

void SunPointingKF(MatrixMath_s* dipoleComm, MatrixMath_s angrate_s, MatrixMath_s r_e2s,  Quaternion_s qEst,
                   MatrixMath_s b_sEst, MatrixMath_s Inertia_ref, ctrlgains_s* ctrlgains, MatrixMath_s angrate);

void SunPointingRaw(MatrixMath_s* dipoleComm, MatrixMath_s angrate_s, MatrixMath_s b_sEst,
                    MatrixMath_s sun_vec_meas, MatrixMath_s Inertia_ref, ctrlgains_s* ctrlgains,
                    MatrixMath_s angrate);

#endif  // HEADER_CONTROL_H_
