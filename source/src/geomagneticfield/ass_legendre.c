#include "geomagnetic_field.h"
#include "math_pack.h"
#include <math.h>

double AssLegendre(double n, double m, double x) {
    double p = 0, p_i = 0;
    int i, length;

    length = floor((n - m) / 2);
    for (i = 0; i <= length; i++) {
        p_i = factorial(2 * n - 2 * i) / factorial(n - i) / factorial(i) /
        factorial(n - 2 * i - m) * pow(-1, i) * pow(x, (n - 2 * i - m));
        p = p + p_i;
    }
    p = p * pow((1 - pow(x, 2)), (m / 2)) * pow((-1), m) / pow(2, n);
    return p;
}
