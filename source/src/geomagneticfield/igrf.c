#include "geomagnetic_field.h"
#include "igrf_data.h"
#include "math_pack.h"
#include <math.h>

void Igrf(igrf_output* igrf_result, double t, MatrixMath_s r_eci, datetime dt, double igrf_order) {
  MatrixMath_s A_i2e = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 1},
    3, 3
  };
  MatrixMath_s A_e2n = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 3
  };
  MatrixMath_s r_ecef = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s b_nedLow = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s b_nedTrue = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s operationTemp3x3 = {
    {3, 0, 0,
     0, 2, 0,
     0, 0, 1},
    3, 3
  };
  MatrixMath_s b_eciTemp = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
    3, 1
  };
  MatrixMath_s b_eciLowerTemp = {
    {0, 0, 0,
     0, 0, 0,
     0, 0, 0},
     3, 1
  };
  double jd = 0, T_uti = 0, theta = 0, r = 0, colat = 0, long_ = 0;
  double jd0 = 0, xlgndr = 0, time = 0, x = 0, y = 0, z = 0, x_nm = 0, y_nm = 0, z_nm = 0, g_nm = 0, h_nm = 0;
  double m, n, xLow, yLow, zLow, xTrue, yTrue, zTrue;
  double lat;

  int year0 = 2010;
  double a = 6371.2;
  int index = 0;

  jd = 1721013.5 + 367 * dt.year - floor(7/4 * (dt.year + floor((dt.month + 9) / 12))) + floor(275 * dt.month / 9) + dt.day;

  T_uti = (jd - 2451545) / 36525;

  theta = 24110.54841 + 8640184.812866 * T_uti + 0.093104 * pow(T_uti, 2) - 6.2 * pow(10, -6) * pow(T_uti, 3) + 1.002737909350795 * (3600 * dt.hour + 60 * dt.min + dt.sec + t);
  theta = theta - floor(theta / 86400) * 86400;

  theta = theta * M_PI / 43200;

  A_i2e.arr[0] = roundTest(cos(theta));
  A_i2e.arr[1] = roundTest(sin(theta));
  A_i2e.arr[3] = roundTest((-1) * A_i2e.arr[1]);
  A_i2e.arr[4] = roundTest(A_i2e.arr[0]);

  multiplyMatrices(&A_i2e, &r_eci, &r_ecef);

  r = vectorNorm(&r_ecef);
  colat = acos(r_ecef.arr[6] / r);
  long_ = atan2(r_ecef.arr[3], r_ecef.arr[0]);

  jd0 = 1721013.5 + 367*year0 - floor(7/4 * (year0 + floor((1 + 9)/12))) + floor(275 * 1/9) + 1;
  jd = jd + (60 * dt.hour + dt.min + (dt.sec + t) / 60) / 1440;
  time = (jd - jd0) * 100 / 36525;

  xlgndr = cos(colat);
  index = 0;

  for (n = 1; n <= igrf_order-1; n++) {
    for (m = 0; m <= n; m++) {
      g_nm = coeffs[index][0] + time * coeffs[index][1];
      h_nm = coeffs[index][2] + time * coeffs[index][3];

      x_nm = pow((a/r), (n + 1)) * (g_nm * cos(m * long_) + h_nm * sin(m * long_)) * DerivLegendre(n, m, xlgndr);
      y_nm = pow((a/r), (n + 1)) * (-g_nm * sin(m * long_) * m + h_nm * cos(m * long_) * m) * SchLegendre(n, m, xlgndr);
      z_nm = (n + 1) * pow((a/r), n) * (-a /(r*r)) * (g_nm * cos(m * long_) + h_nm * sin(m * long_)) * SchLegendre(n, m, xlgndr);

      x = x + x_nm;
      y = y + y_nm;
      z = z + z_nm;
      index = index + 1;
    }
  }

  xLow = -x * sin(colat) * a / r;
  yLow = -y * a / r / sin(colat);
  zLow = z * a;
  b_nedLow.arr[0] = xLow;
  b_nedLow.arr[3] = yLow;
  b_nedLow.arr[6] = zLow;

  n = igrf_order;
  for (m = 0; m <= igrf_order; m++) {
    g_nm = coeffs[index][0] + time * coeffs[index][1];
    h_nm = coeffs[index][2] + time * coeffs[index][3];

    x_nm = pow((a/r), (n + 1)) * (g_nm * cos(m * long_) + h_nm * sin(m * long_)) * DerivLegendre(n, m, xlgndr);
    y_nm = pow((a/r), (n + 1)) * (-g_nm * sin(m * long_) * m + h_nm * cos(m * long_) * m) * SchLegendre(n, m, xlgndr);
    z_nm = (n + 1) * pow((a/r), n) * (-a /(r*r)) * (g_nm * cos(m * long_) + h_nm * sin(m * long_)) * SchLegendre(n, m, xlgndr);

    x = x + x_nm;
    y = y + y_nm;
    z = z + z_nm;
    index = index + 1;
  }

  xTrue = -x * sin(colat) * a / r;
  yTrue = -y * a / r / sin(colat);
  zTrue = z * a;

  b_nedTrue.arr[0] = xTrue;
  b_nedTrue.arr[3] = yTrue;
  b_nedTrue.arr[6] = zTrue;

  lat = M_PI/2 - colat;
  A_e2n.arr[0] = roundTest(-sin(lat)*cos(long_));
  A_e2n.arr[1] = roundTest(-sin(lat)*sin(long_));
  A_e2n.arr[2] = roundTest(cos(lat));

  A_e2n.arr[3] = roundTest(-sin(long_));
  A_e2n.arr[4] = roundTest(cos(long_));
  A_e2n.arr[5] = 0;
  A_e2n.arr[6] = roundTest(-cos(lat)*cos(long_));
  A_e2n.arr[7] = roundTest(-cos(lat)*sin(long_));
  A_e2n.arr[8] = roundTest(-sin(lat));

  transpose(&A_i2e);
  transpose(&A_e2n);

  multiplyMatrices(&A_i2e, &A_e2n, &operationTemp3x3);
  multiplyMatrices(&operationTemp3x3, &b_nedTrue, &b_eciTemp);
  multiplyMatrices(&operationTemp3x3, &b_nedLow, &b_eciLowerTemp);

  (*igrf_result).b_eci = b_eciTemp;
  (*igrf_result).b_eciLower = b_eciLowerTemp;
}
