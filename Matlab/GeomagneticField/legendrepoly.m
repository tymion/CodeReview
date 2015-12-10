function p = legendrepoly(n, x)
% Calculate Legendre polynomial of degree n for x

p = 0;
for i = 0:floor(n/2)
    p_i = factorial(2*n - 2*i) / factorial(n - i) / factorial(i) / factorial(n - 2*i) * (-1)^i * x^(n - 2*i);
    p = p + p_i;
end

p = p / 2^n;