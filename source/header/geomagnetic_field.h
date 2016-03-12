#ifndef HEADER_GEOMAGNETIC_FIELD_H_
#define HEADER_GEOMAGNETIC_FIELD_H_

#include "structurs.h"

double AssLegendre(double n, double m, double x);

double DerivLegendre(double n, double m, double x);

double Legendrepoly(double n,  double x);

double SchLegendre(double n, double m, double x);

void Igrf(igrf_output* igrf_result, double t, MatrixMath_s r_eci, datetime dt, double igrf_order);

#endif  // HEADER_GEOMAGNETIC_FIELD_H_
