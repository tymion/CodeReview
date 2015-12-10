function A = skew(w)

% Calculating the 3x3 skew-symmetric matrix based on 3-element vector w
A = zeros(3);

A(1,2) = -w(3);
A(1,3) = w(2);

A(2,1) = w(3);
A(2,3) = -w(1);

A(3,1) = -w(2);
A(3,2) = w(1);

end