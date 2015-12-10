void WahbaTriadWeight(MatrixMath_s* result, MatrixMath_s b1, MatrixMath_s b2, MatrixMath_s r1, MatrixMath_s r2,
                                   double sigmaWahba_b1, double sigmaWahba_b2, double sigmaWahba_r1, double sigmaWahba_r2) {
  MatrixMath_s skewb = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s skewr = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s tmpb = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s tmpr = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s b_x = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s r_x = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s b1r1 = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s b2r2 = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s b1bx = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s b2bx = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s r1rx = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s r2rx = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s bxrx = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s tmp_sum_1 = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s tmp_sum_2 = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s tmp_sum_3 = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s tmp_multiply_1 = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s tmp_multiply_2 = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  double vecNormb = 0;
  double vecNormr = 0;
  double lambda_max = 0;
  double sigma1 = sqrt(pow(sigmaWahba_r1, 2) + pow(sigmaWahba_b1, 2));
  double sigma2 = sqrt(pow(sigmaWahba_r2, 2) + pow(sigmaWahba_b2, 2));

  double c = 1/(1/pow(sigma1, 2) + 1/pow(sigma2, 2));
  double a1 = c / pow(sigma1, 2);
  double a2 = c / pow(sigma2, 2);

  skew(b1, &skewb);
  skew(r1, &skewr);

  multiplyMatrices(&skewb, &b2, &tmpb);
  multiplyMatrices(&skewb, &b2, &b_x);
  multiplyMatrices(&skewr, &r2, &tmpr);
  multiplyMatrices(&skewr, &r2, &r_x);

  vecNormb = vectorNorm(&tmpb);
  vecNormr = vectorNorm(&tmpr);
  multiplyScalar(1/vecNormb, &b_x);
  multiplyScalar(1/vecNormr, &r_x);

  transpose(&b1);
  transpose(&r1);

  multiplyMatrices(&b1, &b2, &tmpb);
  multiplyMatrices(&r1, &r2, &tmpr);

  lambda_max = sqrt(pow(a1, 2) + pow(a2, 2) + 2*a1*a2 * (tmpb.arr[0] * tmpr.arr[0] + vecNormb * vecNormr));

  transpose(&b1);
  transpose(&r1);

  skew(b1, &skewb);
  multiplyMatrices(&skewb, &b_x, &b1bx);
  skew(r1, &skewr);
  multiplyMatrices(&skewr, &r_x, &r1rx);
  transpose(&r1rx);

  resetMatrix(&skewb);
  resetMatrix(&skewr);
  skew(b2, &skewb);
  multiplyMatrices(&skewb, &b_x, &b2bx);
  skew(r2, &skewr);
  multiplyMatrices(&skewr, &r_x, &r2rx);
  transpose(&r2rx);

  multiplyMatrices(&b1bx, &r1rx, &tmp_multiply_1);
  multiplyMatrices(&b2bx, &r2rx, &tmp_multiply_2);

  transpose(&r1);
  transpose(&r2);
  transpose(&r_x);

  multiplyMatrices(&b1, &r1, &b1r1);
  multiplyMatrices(&b2, &r2, &b2r2);
  multiplyMatrices(&b_x, &r_x, &bxrx);

  addingMatrices(&b1r1, &tmp_multiply_1, &tmp_sum_1);
  addingMatrices(&b2r2, &tmp_multiply_2, &tmp_sum_2);

  multiplyScalar(a1/lambda_max, &tmp_sum_1);
  multiplyScalar(a2/lambda_max, &tmp_sum_2);

  addingMatrices(&tmp_sum_1, &tmp_sum_2, &tmp_sum_3);
  addingMatrices(&tmp_sum_3, &bxrx, result);
}
