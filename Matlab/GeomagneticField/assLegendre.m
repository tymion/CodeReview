function p = assLegendre(n, m, x)
% Calculate associated Legendre polynomial of degree n and order m for x

p = 0;
for i = 0:floor((n - m)/2)   
    i;
    p_i = factorial(2*n - 2*i) / factorial(n - i) / factorial(i) / factorial(n - 2*i - m) * (-1)^i * x^(n - 2*i - m);
    p_i;
    p = p + p_i;
end

p = p * (1 - x^2)^(m/2) * (-1)^m / 2^n;