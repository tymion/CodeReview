function norm = vectorNorm(w)
% Calculating norm of a input vector w

size = length(w);
sum = 0;

for i = 1:size
    sum_i = w(i) * w(i);
    sum = sum + sum_i;
end

norm = sqrt(sum);

end