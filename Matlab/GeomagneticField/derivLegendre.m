function d = derivLegendre(n, m, x)
% Calculate derivative of Schmidt seminormalized associated Legendre polynomial with respect to x

if m == 0
    d = -1 * assLegendre(n, 1, x) / sqrt(1 - x^2);
else
    d_a = sqrt(2 * factorial(n - m) / factorial(n + m));
    d_b = -m * x / (1 - x^2) * (-1)^m * assLegendre(n,m,x);
    
    if m == n
        d_c = 0;
    else
        d_c = (-1)^(m + 1) / sqrt(1 - x^2) * assLegendre(n, m+1, x);
    end
    
    d = d_a * (d_b + d_c);
end