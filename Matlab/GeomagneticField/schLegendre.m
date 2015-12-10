function p = schLegendre(n, m, x)
% Calculate Schmidt seminormalized associated Legendre polynomial

if m == 0
    p = legendrepoly(n, x);
elseif m > 0
    p = (-1)^m * sqrt(2 * factorial(n - m) / factorial(n + m)) * assLegendre(n,m,x);
else
    error('Order m cannot be negative')
end

