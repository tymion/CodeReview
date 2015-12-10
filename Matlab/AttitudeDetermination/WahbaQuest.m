function q = WahbaQuest(b1, b2, r1, r2)
% ******************************************************
% WAHBA SOLUTION FOR 2 OBSERVATIONS - QUEST
% measurement vectors expressed in satellite frame {b1, b2}
% reference vectors expressed in ECI frame {r1, r2}
% b1 & r1 refers to magnetic field measurement
% b2 & r2 refers to sun vector measurement
% b1, b2, r1, r2 are column unit vectors
%
% sigma_i^2 = sigma_ri^2 + sigma_bi^2
% where 
% i = {1,2}
% r, b indexes refer to reference and body respectively
% ******************************************************

global sigmaWahba;

%% Define weights for 2 observation vectors in Wahba problem
% combined standard deviation of reference and body vectors
sigma1 = sqrt(sigmaWahba(1).r^2 + sigmaWahba(1).b^2);
sigma2 = sqrt(sigmaWahba(2).r^2 + sigmaWahba(2).b^2);

c = (1 / sigma1^2 + 1 / sigma2^2)^(-1);
a1 = c / sigma1^2;
a2 = c / sigma2^2;

%% Optimal quaternion based on 2 observations
% greatest eigenvalue of K(B) matrix in Wahba solution
% lambda_max = sqrt(a1^2 + a2^2 + 2*a1*a2 * ((b1'*b2)*(r1'*r2) + vectorNorm(skew(b1)*b2) * vectorNorm(skew(r1)*r2)));

B = a1*b1*r1' + a2*b2*r2';
S = B + B';
z = [B(2,3) - B(3,2), B(3,1) - B(1,3), B(1,2) - B(2,1)]';
k = trace(S \ eye(3) * det(S));

% Newton-Raphson procedure to find greatest eigenvalue of K(B)
threshold = 1e-8;
lambda = a1 + a2;
fcn = (lambda^2 - trace(B)^2 + k) * (lambda^2 - trace(B)^2 - vectorNorm(z)^2) ...
    - (lambda - trace(B)) * (z'*S*z + det(S)) - z' * S^2 * z;

if abs(fcn) > threshold
    while abs(fcn) > threshold
        deriv_fcn = 4*lambda^3 + 2*lambda*(k - 2*trace(B)^2 - vectorNorm(z)^2) - z'*S*z - det(S);            
        delta_lambda = fcn / deriv_fcn;
        lambda = lambda - delta_lambda;
        fcn = (lambda^2 - trace(B)^2 + k) * (lambda^2 - trace(B)^2 - vectorNorm(z)^2) ...
            - (lambda - trace(B)) * (z'*S*z + det(S)) - z' * S^2 * z;
    end % end while
end % end if

lambda_max = lambda;
ro = lambda_max + trace(B);

q = [(ro * eye(3) - S) \ z * det(ro * eye(3) - S); det(ro * eye(3) - S)];
q = qnorm(q);
