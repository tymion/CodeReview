function q = m2q(A)
% Extract quaternion from attitude matrix
% Avoid numerical errors by choosing the quaternion with the greatest
% norm

a = [trace(A), A(1,1), A(2,2), A(3,3)];
max_value = max(a);

if max_value == A(1,1)
    q(1) = 0.5 * sqrt(1 + 2*A(1,1) - trace(A));
    q(2) = (A(1,2) + A(2,1)) / 4 / q(1);
    q(3) = (A(1,3) + A(3,1)) / 4 / q(1);
    q(4) = (A(2,3) - A(3,2)) / 4 / q(1);
elseif max_value == A(2,2)
    q(2) = 0.5 * sqrt(1 + 2*A(2,2) - trace(A));
    q(1) = (A(1,2) + A(2,1)) / 4 / q(2);
    q(3) = (A(2,3) + A(3,2)) / 4 / q(2);
    q(4) = (A(3,1) - A(1,3)) / 4 / q(2);
elseif max_value == A(3,3)
    q(3) = 0.5 * sqrt(1 + 2*A(3,3) - trace(A));
    q(1) = (A(3,1) + A(1,3)) / 4 / q(3);
    q(2) = (A(2,3) + A(3,2)) / 4 / q(3);
    q(4) = (A(1,2) - A(2,1)) / 4 / q(3);
elseif max_value == trace(A)
    q(4) = 0.5 * sqrt(1 + trace(A));
    q(1) = (A(2,3) - A(3,2)) / 4 / q(4);
    q(2) = (A(3,1) - A(1,3)) / 4 / q(4);
    q(3) = (A(1,2) - A(2,1)) / 4 / q(4);
end

q = q';