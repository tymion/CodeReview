void Detumbling(MatrixMath_s* commDipole, MatrixMath_s bMeas, MatrixMath_s bDeriv, double ctrlgains_bdot) {
  double temp_1e9 = pow(10, -9);
  double temp = ctrlgains_bdot * temp_1e9;
  double r = 0;

  multiplyScalar(temp * (-1), &bDeriv);
  multiplyScalar(temp_1e9, &bMeas);
  r = pow(vectorNorm(&bMeas), 2);

  multiplyScalar(1/r, &bDeriv);
  (*commDipole) = bDeriv;
}
