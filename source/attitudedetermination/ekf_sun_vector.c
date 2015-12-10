void EkfSunVector(MatrixMath7x1_s* result7x1, MatrixMath6x6_s* result6x6, MatrixMath9x1_s* result9x1, MatrixMath9x9_s* result9x9,
                           MatrixMath7x1_s X_k, MatrixMath6x6_s P_k, MatrixMath9x1_s Z, double eclipse, MatrixMath_s b_eci, MatrixMath_s r_e2s_eci,
                           double time_step, MatrixMath9x9_s KalmanCovariance_R, MatrixMath6x6_s KalmanCovariance_Q, MatrixMath_s Inertia_ref,
                           MatrixMath_s ctrlTorque_prev) {
  Quaternion_s q_k = {{
    X_k.arr[0],
    X_k.arr[1],
    X_k.arr[2],
    X_k.arr[3],
  }};

  MatrixMath_s angrate_k = {{
    X_k.arr[4], 0, 0,
    X_k.arr[5], 0, 0,
    X_k.arr[6], 0, 0},
    3, 1
  };

  MatrixMath7x1_s k_tmp = {{
    0, 0, 0, 0, 0, 0, 0
  }};
  MatrixMath7x1_s k1 = {{
    0, 0, 0, 0, 0, 0, 0
  }};
  MatrixMath7x1_s k2 = {{
    0, 0, 0, 0, 0, 0, 0
  }};
  MatrixMath7x1_s k3 = {{
    0, 0, 0, 0, 0, 0, 0
  }};
  MatrixMath7x1_s k4 = {{
    0, 0, 0, 0, 0, 0, 0
  }};
  MatrixMath7x1_s stateVectorPredict = {{
    0, 0, 0, 0, 0, 0, 0
  }};
  MatrixMath7x1_s tmp_sum1 = {{
    0, 0, 0, 0, 0, 0, 0
  }};
  MatrixMath7x1_s tmp_sum2 = {{
    0, 0, 0, 0, 0, 0, 0
  }};
  MatrixMath7x1_s tmp_sum3 = {{
    0, 0, 0, 0, 0, 0, 0
  }};
  double scalar_k1_k2 = time_step * 0.5;
  Quaternion_s qPredict = {{
    0, 0, 0, 0
  }};
  MatrixMath_s tmp_angrate_k = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 3};
  MatrixMath_s tmp_mult_1 = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 1};
  MatrixMath_s tmp_mult_2 = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 3};
  MatrixMath_s tmp_skew_1 = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 3};
  MatrixMath_s tmp_subs_1 = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 3};
  MatrixMath_s tmp_inver_1 = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 3};
  MatrixMath6x6_s jacobianF = {{
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0},
    6, 6
  };
  MatrixMath6x6_s stateTrans = {{
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0},
    6, 6
  };
  MatrixMath6x6_s eye6 = {{
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1},
    6, 6
  };

  MatrixMath6x6_s tmp6x6_multi_1 = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 6, 6};
  MatrixMath6x6_s tmp6x6_multi_2 = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 6, 6};
  MatrixMath6x6_s P_predict = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 6, 6};
  MatrixMath_s A_i2sPredict = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 3};
  MatrixMath_s magFieldPredict = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 1};
  MatrixMath_s magFieldMeas = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 1};
  MatrixMath_s sunVectorPredict = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 1};
  MatrixMath_s sunVectorMeas = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 1};
  MatrixMath_s angratePredict = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 1};
  MatrixMath_s angrateMeas = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 1};
  MatrixMath_s skewMagFieldPredict = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 3};
  MatrixMath_s skewSunVectorPredict = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 3, 3};
  MatrixMath9x6_s H = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 9, 6};
  MatrixMath9x6_s H_t = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 6, 9};
  MatrixMath9x6_s resCov_tmp1 = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 9, 6};
  MatrixMath9x9_s resCov_tmp2 = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 9, 9};
  MatrixMath9x9_s resCov = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 9, 9};
  MatrixMath9x9_s K_tmpInverse = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 9, 9};
  MatrixMath9x6_s K_tmp1 = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 6, 9};
  MatrixMath9x9_s K = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 6, 9};

  PropagateState(&k1, X_k, ctrlTorque_prev, Inertia_ref);

  multiplyScalar7x1(scalar_k1_k2, &k1);
  addingMatrices7x1(&X_k, &k1, &k_tmp);
  PropagateState(&k2, k_tmp, ctrlTorque_prev, Inertia_ref);

  multiplyScalar7x1(scalar_k1_k2, &k2);
  addingMatrices7x1(&X_k, &k2, &k_tmp);
  PropagateState(&k3, k_tmp, ctrlTorque_prev, Inertia_ref);

  multiplyScalar7x1(time_step, &k3);
  addingMatrices7x1(&X_k, &k3, &k_tmp);
  PropagateState(&k4, k_tmp, ctrlTorque_prev, Inertia_ref);

  multiplyScalar7x1(2, &k2);
  multiplyScalar7x1(2, &k3);
  addingMatrices7x1(&k1, &k2, &tmp_sum1);
  addingMatrices7x1(&tmp_sum1, &k3, &tmp_sum2);
  addingMatrices7x1(&tmp_sum2, &k4, &tmp_sum3);
  multiplyScalar7x1(time_step/6, &tmp_sum3);
  addingMatrices7x1(&X_k, &tmp_sum3, &stateVectorPredict);

  qPredict.arr[0] = stateVectorPredict.arr[0];
  qPredict.arr[1] = stateVectorPredict.arr[1];
  qPredict.arr[2] = stateVectorPredict.arr[2];
  qPredict.arr[3] = stateVectorPredict.arr[3];
  q_norm(&qPredict);

  /* Discrete Jacobian for state transition */
  skew(angrate_k, &tmp_angrate_k);
  multiplyMatrices(&Inertia_ref, &angrate_k, &tmp_mult_1);
  skew(tmp_mult_1, &tmp_skew_1);
  multiplyMatrices(&tmp_angrate_k, &Inertia_ref, &tmp_mult_2);
  subtractionMatrices(&tmp_skew_1, &tmp_mult_2, &tmp_subs_1);
  inverse(&Inertia_ref);
  multiplyMatrices(&Inertia_ref, &tmp_subs_1, &tmp_inver_1);
  multiplyScalar(-1, &tmp_angrate_k);

  jacobianF.arr[1] = tmp_angrate_k.arr[1];
  jacobianF.arr[2] = tmp_angrate_k.arr[2];
  jacobianF.arr[3] = 1.0;

  jacobianF.arr[6] = tmp_angrate_k.arr[3];
  jacobianF.arr[8] = tmp_angrate_k.arr[5];
  jacobianF.arr[10] = 1.0;

  jacobianF.arr[12] = tmp_angrate_k.arr[6];
  jacobianF.arr[13] = tmp_angrate_k.arr[7];
  jacobianF.arr[17] = 1.0;

  jacobianF.arr[21] = tmp_inver_1.arr[0];
  jacobianF.arr[22] = tmp_inver_1.arr[1];
  jacobianF.arr[23] = tmp_inver_1.arr[2];

  jacobianF.arr[27] = tmp_inver_1.arr[3];
  jacobianF.arr[28] = tmp_inver_1.arr[4];
  jacobianF.arr[29] = tmp_inver_1.arr[5];

  jacobianF.arr[33] = tmp_inver_1.arr[6];
  jacobianF.arr[34] = tmp_inver_1.arr[7];
  jacobianF.arr[35] = tmp_inver_1.arr[8];

  multiplyScalar6x6(time_step, &jacobianF);
  addingMatrices6x6(&eye6, &jacobianF, &stateTrans);

  multiplyMatrices6x6(&stateTrans, &P_k, &tmp6x6_multi_1);
  transpose6x6(&stateTrans);
  multiplyMatrices6x6(&tmp6x6_multi_1, &stateTrans, &tmp6x6_multi_2);
  multiplyScalar6x6(time_step, &KalmanCovariance_Q);
  addingMatrices6x6(&tmp6x6_multi_2, &KalmanCovariance_Q, &P_predict);
  q2m(qPredict, &A_i2sPredict);

  multiplyMatrices(&A_i2sPredict, &b_eci, &magFieldPredict);

  magFieldMeas.arr[0] = Z.arr[0];
  magFieldMeas.arr[3] = Z.arr[1];
  magFieldMeas.arr[6] = Z.arr[2];

  multiplyMatrices(&A_i2sPredict, &r_e2s_eci, &sunVectorPredict);
  if (eclipse == 0) {
    sunVectorMeas.arr[0] = Z.arr[3];
    sunVectorMeas.arr[3] = Z.arr[4];
    sunVectorMeas.arr[6] = Z.arr[5];
  } else {
    sunVectorMeas = sunVectorPredict;
  }

  angratePredict.arr[0] = stateVectorPredict.arr[4];
  angratePredict.arr[3] = stateVectorPredict.arr[5];
  angratePredict.arr[6] = stateVectorPredict.arr[6];

  angrateMeas.arr[0] = Z.arr[6];
  angrateMeas.arr[3] = Z.arr[7];
  angrateMeas.arr[6] = Z.arr[8];

  skew(magFieldPredict, &skewMagFieldPredict);
  skew(sunVectorPredict, &skewSunVectorPredict);

  H.arr[0] = skewMagFieldPredict.arr[0];
  H.arr[1] = skewMagFieldPredict.arr[1];
  H.arr[2] = skewMagFieldPredict.arr[2];
  H.arr[3] = 0;
  H.arr[4] = 0;
  H.arr[5] = 0;
  H.arr[6] = skewMagFieldPredict.arr[3];
  H.arr[7] = skewMagFieldPredict.arr[4];
  H.arr[8] = skewMagFieldPredict.arr[5];
  H.arr[9] = 0;
  H.arr[10] = 0;
  H.arr[11] = 0;
  H.arr[12] = skewMagFieldPredict.arr[6];
  H.arr[13] = skewMagFieldPredict.arr[7];
  H.arr[14] = skewMagFieldPredict.arr[8];
  H.arr[15] = 0;
  H.arr[16] = 0;
  H.arr[17] = 0;
  H.arr[18] = skewSunVectorPredict.arr[0];
  H.arr[19] = skewSunVectorPredict.arr[1];
  H.arr[20] = skewSunVectorPredict.arr[2];
  H.arr[21] = 0;
  H.arr[22] = 0;
  H.arr[23] = 0;
  H.arr[24] = skewSunVectorPredict.arr[3];
  H.arr[25] = skewSunVectorPredict.arr[4];
  H.arr[26] = skewSunVectorPredict.arr[5];
  H.arr[27] = 0;
  H.arr[28] = 0;
  H.arr[29] = 0;
  H.arr[30] = skewSunVectorPredict.arr[6];
  H.arr[31] = skewSunVectorPredict.arr[7];
  H.arr[32] = skewSunVectorPredict.arr[8];
  H.arr[33] = 0;
  H.arr[34] = 0;
  H.arr[35] = 0;
  H.arr[36] = 0;
  H.arr[37] = 0;
  H.arr[38] = 0;
  H.arr[39] = 1;
  H.arr[40] = 0;
  H.arr[41] = 0;
  H.arr[42] = 0;
  H.arr[43] = 0;
  H.arr[44] = 0;
  H.arr[45] = 0;
  H.arr[46] = 1;
  H.arr[47] = 0;
  H.arr[48] = 0;
  H.arr[49] = 0;
  H.arr[50] = 0;
  H.arr[51] = 0;
  H.arr[52] = 0;
  H.arr[53] = 1;

  H_t = H;
  H_t.row = 6;
  H_t.col = 9;
  transpose9x6(&H_t);

  multiplyMatrices9x6and6x6(&H, &P_predict, &resCov_tmp1);
  multiplyMatrices9x6and6x9(&resCov_tmp1, &H_t, &resCov_tmp2);
  addingMatrices9x9(&resCov_tmp2, &KalmanCovariance_R, &resCov);

  multiplyMatrices6x6and9x6(&P_predict, &H_t, &K_tmp1);
  // TODO(jkwietko): inverse matrix 9x9
  // multiplyMatrices9x6and9x9(&K_tmp1, &K_tmpInverse, &K);
  
  if (eclipse == 1) {
    
  }    
}
