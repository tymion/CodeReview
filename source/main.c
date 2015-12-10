#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "header/main.h"
#include "header/math_pack.h"
#include "header/debug_pack.h"
#include "header/geomagnetic_field.h"
#include "header/control.h"
#include "header/attitude_determination.h"

int main(void) {
  system("cls");
  printf("<! ------------------------------\n\n");

  printf("#### GeomagneticField:\n");
  TestAssLegendre();
  TestDerivLegendre();
  TestSchLegendre();
  TestLegendrepoly();
  TestIgrf();

  printf("#### Control:\n");
  TestDetumbling();
  TestSunPointingKF();
  TestSunPointingRaw();

  printf("#### AtitudeDetermination:\n");
  TestWahbaTriadWeight();
  TestPropagateState();

  printf("\n--------------------------------!>");
  return 0;
}

void TestAssLegendre(void) {
  double result;
  /*!< execute */
  result = AssLegendre(5, 2, 0.5);
  printf("\tAssLegendre = %f\n", result);
}

void TestDerivLegendre(void) {
  double result;
  /*!< execute */
  result = DerivLegendre(4, 1, 0.6);
  printf("\tDerivLegendre = %f\n", result);
}

void TestLegendrepoly(void) {
  double result;
  /*!< execute */
  result = Legendrepoly(6, -0.4);
  printf("\tLegendrepoly = %f\n", result);
}

void TestSchLegendre(void) {
  double result;
  /*!< execute */
  result = SchLegendre(5, 0, -0.9);
  printf("\tSchLegendre = %f\n", result);
}

void TestIgrf(void) {
  igrf_output igrf_result;
  double t = 0, igrf_order = 0;
  datetime dt;
  MatrixMath_s r_eci = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    1, 3
  };

  dt.year = 2015;
  dt.month = 5;
  dt.day = 24;
  dt.hour = 18;
  dt.min = 0;
  dt.sec = 0;

  t = 100;
  igrf_order = 12;

  r_eci.col = 1;
  r_eci.row = 3;
  r_eci.arr[0] = 6800;
  r_eci.arr[3] = -120;
  r_eci.arr[6] = 1000;

  /*!< execute */
  Igrf(&igrf_result, t, r_eci, dt, igrf_order);
  printf("\tigrf  = %.3f;%.3f;%.3f;%.3f;%.3f;%.3f;\n", igrf_result.b_eci.arr[0],
                                                                          igrf_result.b_eci.arr[3],
                                                                          igrf_result.b_eci.arr[6],
                                                                          igrf_result.b_eciLower.arr[0],
                                                                          igrf_result.b_eciLower.arr[3],
                                                                          igrf_result.b_eciLower.arr[6]);
}

void TestDetumbling(void) {
  double ctrlgains_bdot = 0;
  MatrixMath_s commDipole = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s bMeas = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s bDeriv = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  bMeas.arr[0] = 1;
  bMeas.arr[3] = 2;
  bMeas.arr[6] = 3;

  bDeriv.arr[0] = -2;
  bDeriv.arr[3] = 0.1;
  bDeriv.arr[6] = -1;

  ctrlgains_bdot = 2;

  /*!< execute */
  Detumbling(&commDipole, bMeas, bDeriv, ctrlgains_bdot);
  printf("\tDetumbling = %.3f;%.3f;%.3f;\n", commDipole.arr[0],
                                                                commDipole.arr[3],
                                                                commDipole.arr[6]);
}

void TestSunPointingRaw(void) {
  MatrixMath_s dipoleComm = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s angrate_s = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s b_sEst = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s sun_vec_meas = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s Inertia_ref = {
    {1, 2, 3,
     4, 5, 6,
     7, 8, 9},
    3, 3
  };
  MatrixMath_s angrate = {
    {0.1, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  ctrlgains_s ctrlgains = {
    0, 0, 0
  };
  angrate_s.arr[0] = 0.1;
  angrate_s.arr[3] = 1;
  angrate_s.arr[6] = -2;

  b_sEst.arr[0] = 1;
  b_sEst.arr[3] = 2;
  b_sEst.arr[6] = 3;

  sun_vec_meas.arr[0] = 1;
  sun_vec_meas.arr[3] = 0;
  sun_vec_meas.arr[6] = 0;

  ctrlgains.k = 4*pow(10, -3);
  ctrlgains.kp = 4*pow(10, -3);
  ctrlgains.kn = -1*pow(10, -4);

  /*!< execute */
  SunPointingRaw(&dipoleComm, angrate_s, b_sEst, sun_vec_meas, Inertia_ref, &ctrlgains, angrate);
  printf("\tSunPointingRaw = %.10f;%.10f;%.10f;\n", dipoleComm.arr[0],
                                                                            dipoleComm.arr[3],
                                                                            dipoleComm.arr[6]);
}

void TestSunPointingKF(void) {
  MatrixMath_s dipoleComm = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s angrate_s = {
    {0.1, 0, 0,
     1, 0, 0,
     -2, 0, 0},
    3, 1
  };
  MatrixMath_s r_e2s = {
    {0.3369, 0, 0,
     0.8422, 0, 0,
    -0.4211, 0, 0},
    3, 1
  };
  Quaternion_s qEst = {
    {0, 0, 0, 0}
  };
  MatrixMath_s b_sEst = {
    {1, 0, 0,
     2, 0, 0,
     3, 0, 0},
    3, 1
  };
  MatrixMath_s Inertia_ref = {
    {1, 2, 3,
     4, 5, 6,
     7, 8, 9},
    3, 3
  };
  MatrixMath_s angrate = {
    {0.1, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  double a = 4*pow(10, -3);
  double b = -1*pow(10, -4);
  ctrlgains_s ctrlgains = {
    0, 0, 0
  };
  ctrlgains.k = a;
  ctrlgains.kp = a;
  ctrlgains.kn = b;

  qEst.arr[1] = sin(30*M_PI/180);
  qEst.arr[3] = cos(30*M_PI/180);

  /*!< execute */
  SunPointingKF(&dipoleComm, angrate_s, r_e2s, qEst, b_sEst, Inertia_ref, &ctrlgains, angrate);
  printf("\tSunPointingKF = %.10f;%.10f;%.10f;\n", dipoleComm.arr[0],
                                                                          dipoleComm.arr[3],
                                                                          dipoleComm.arr[6]);
}

void TestWahbaTriadWeight(void) {
  MatrixMath_s result = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s b1 = {
    {0.5148, 0, 0,
     -0.8346, 0, 0,
     -0.1960, 0, 0},
    3, 1
  };
  MatrixMath_s b2 = {
    {-0.2013, 0, 0,
     -0.5187, 0, 0,
     0.8309, 0, 0},
    3, 1
  };
  MatrixMath_s r1 = {
    {0.0447, 0, 0,
     0.4468, 0, 0,
     -0.8935, 0, 0},
    3, 1
  };
  MatrixMath_s r2 = {
    {0.9798, 0, 0,
     0.1960, 0, 0,
     -0.0392, 0, 0},
    3, 1
  };
  double sigmaWahba_b1 = 0.0523;
  double sigmaWahba_b2 = 0.1045;
  double sigmaWahba_r1 = 0.0017;
  double sigmaWahba_r2 = 0.0052;

  WahbaTriadWeight(&result, b1, b2, r1, r2, sigmaWahba_b1, sigmaWahba_b2, sigmaWahba_r1, sigmaWahba_r2);
  printf("\tWahbaTriadWeight = %f;%f;%f;%f;%f;%f;%f;%f;%f;\n", result.arr[0],
                                                                                            result.arr[1],
                                                                                            result.arr[2],
                                                                                            result.arr[3],
                                                                                            result.arr[4],
                                                                                            result.arr[5],
                                                                                            result.arr[6],
                                                                                            result.arr[7],
                                                                                            result.arr[8]);
}

void TestPropagateState() {
  MatrixMath7x1_s result_s = {{
    0, 0, 0, 0, 0, 0, 0
  }};
  MatrixMath7x1_s z = {{
    0.2315,
    0.4629,
    0.6944,
    0.5000,
    0.1100,
    0.2200,
    -0.1500
  }};
  MatrixMath_s ctrlTorque_prev = {
    {1, 0, 0,
     2, 0, 0,
     3, 0, 0},
    3, 1
  };
  MatrixMath_s Inertia_ref = {
    {1, 2, 3,
     2, 5, 1,
     3, 1, 9},
    3, 3
  };

  multiplyScalar(pow(10, -8), &ctrlTorque_prev);
  multiplyScalar(pow(10, -4), &Inertia_ref);

  PropagateState(&result_s, z, ctrlTorque_prev, Inertia_ref);
  printf("\tPropagateState = %.10f;%.10f;%.10f;%.10f;%.10f;%.10f;%.10f;\n", result_s.arr[0],
                                                                                                                 result_s.arr[1],
                                                                                                                 result_s.arr[2],
                                                                                                                 result_s.arr[3],
                                                                                                                 result_s.arr[4],
                                                                                                                 result_s.arr[5],
                                                                                                                 result_s.arr[6]);
}


void TestEkfSunVector(void) {
  MatrixMath7x1_s result7x1 = {
    {0, 0, 0, 0, 0, 0, 0}
  };
  MatrixMath6x6_s result6x6 = {
    {0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0},
    6, 6
  };
  MatrixMath9x1_s result9x1 = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0}
  };
  MatrixMath9x9_s result9x9 = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0},
    9, 9
  };
  double eclipse = 0.0;
  double time_step = 1.0;

  MatrixMath7x1_s X_k = {{
    0.2315,
    0.4629,
    0.6944,
    0.5000,
    0.1100,
    0.2200,
    -0.1500
  }};

  MatrixMath6x6_s P_k = {{
      1.0*pow(10, -6), 0, 0, 0, 0, 0,
      0, 2.0*pow(10, -6), 0, 0, 0, 0,
      0, 0, 6.0*pow(10, -6), 0, 0, 0,
      0, 0, 0, 2.0*pow(10, -6), 0, 0,
      0, 0, 0, 0, 3.0*pow(10, -6), 0,
      0, 0, 0, 0, 0, 9.0*pow(10, -6)
    },
    6, 6
  };

  MatrixMath9x1_s Z = {{
    1.0,
    2.0,
    3.0,
    -1.0,
    2.0,
    0.5,
    0.1,
    0.2,
    -0.1
  }};

  MatrixMath_s b_eci = {
    {1, 0, 0,
     2, 0, 0,
     3, 0, 0},
    3, 1
  };

  MatrixMath_s r_e2s_eci = {
    {0, 0, 0,
     0.8944, 0, 0,
     -0.4472, 0, 0},
    3, 1
  };

  MatrixMath9x9_s KalmanCovariance_R = {{
    1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 2, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 2, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 4, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 4, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 4},
    9, 9
  };

  MatrixMath6x6_s KalmanCovariance_Q = {{
    pow(10, -8), 0, 0, 0, 0, 0,
    0, pow(10, -8), 0, 0, 0, 0,
    0, 0, pow(10, -8), 0, 0, 0,
    0, 0, 0, pow(10, -7), 0, 0,
    0, 0, 0, 0, pow(10, -7), 0,
    0, 0, 0, 0, 0, pow(10, -7)},
    6, 6
  };

  MatrixMath_s Inertia_ref = {
    {1*pow(10, -4), 2*pow(10, -4), 3*pow(10, -4),
     2*pow(10, -4), 5*pow(10, -4), 1*pow(10, -4),
     3*pow(10, -4), 1*pow(10, -4), 9*pow(10, -4)},
    3, 3
  };

  MatrixMath_s ctrlTorque_prev = {
    {1*pow(10, -8),
    2*pow(10, -8),
    3*pow(10, -8)},
    3, 1
  };

  EkfSunVector(&result7x1, &result6x6, &result9x1, &result9x9, X_k, P_k, Z, eclipse, b_eci, r_e2s_eci, time_step, KalmanCovariance_R, KalmanCovariance_Q, Inertia_ref, ctrlTorque_prev);
}
