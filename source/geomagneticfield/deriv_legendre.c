double DerivLegendre(double n, double m, double x) {
  double d = 0;
  if (m == 0) {
    d = (-1) * AssLegendre(n, 1, x) / sqrt(1-pow(x, 2));
  } else {
    double d_a, d_b, d_c;
    d_a = sqrt(2 * factorial(n-m)/factorial(n+m));
    d_b = (-m)*x/(1-pow(x, 2))*pow(-1, m)*AssLegendre(n, m, x);
    if (m == n)
      d_c = 0;
    else
      d_c = pow(-1, m+1) / sqrt(1-pow(x, 2)) * AssLegendre(n, m+1, x);
    d = d_a * (d_b + d_c);
  }
  return d;
}
