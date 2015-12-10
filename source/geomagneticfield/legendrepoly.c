double Legendrepoly(double n,  double x) {
  int i;
  int floor_n = floor(n/2);
  double p = 0, p_i = 0;
  for (i = 0; i <= floor_n; i++) {
    p_i = factorial(2*n-2*i)/factorial(n-i)/factorial(i)/factorial(n-2*i) * pow(-1, i) * pow(x, n-2*i);
    p = p + p_i;
  }
  p = p/pow(2, n);
  return p;
}
