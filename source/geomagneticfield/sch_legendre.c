double SchLegendre(double n, double m, double x) {
  double p = 0;
  if (m == 0)
    p = Legendrepoly(n, x);
  else if (m > 0)
    p = pow((-1), m) * sqrt(2 * factorial(n-m)/factorial(n+m)) * AssLegendre(n, m, x);
  return p;
}
