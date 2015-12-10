function q_norm = qnorm(q)

% Quaternion normalization
norm = sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);
q_norm = 1 / norm * q;

end