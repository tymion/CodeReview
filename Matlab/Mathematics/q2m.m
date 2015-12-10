function A = q2m(q)

% Calculating rotation matrix from quaternion [q1 q2 q3 q4]' where q4 is
% scalar part
A = (q(4)^2 - vectorNorm(q(1:3))^2) * eye(3) - 2 * q(4) * skew(q(1:3)) + 2 * q(1:3) * q(1:3)';
end