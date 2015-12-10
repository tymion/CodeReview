function A = WahbaTriadWeight(b1, b2, r1, r2)
% ******************************************************
% WAHBA SOLUTION FOR 2 OBSERVATIONS - TRIAD
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
sigma1 = sqrt(sigmaWahba.r(1)^2 + sigmaWahba.b(1)^2);
sigma2 = sqrt(sigmaWahba.r(2)^2 + sigmaWahba.b(2)^2);

c = (1 / sigma1^2 + 1 / sigma2^2)^(-1);
a1 = c / sigma1^2;
a2 = c / sigma2^2;

%% Optimal attitude matrix based on 2 observations 
% greatest eigenvalue of K(B) matrix in Wahba solution
lambda_max = sqrt(a1^2 + a2^2 + 2*a1*a2 * ((b1'*b2)*(r1'*r2) + vectorNorm(skew(b1)*b2) * vectorNorm(skew(r1)*r2)));

b_x = skew(b1) * b2 / vectorNorm(skew(b1) * b2);
r_x = skew(r1) * r2 / vectorNorm(skew(r1) * r2);

(skew(r1)*r_x)'

A = a1/lambda_max * (b1*r1' + (skew(b1)*b_x) * (skew(r1)*r_x)') + ...
    a2/lambda_max * (b2*r2' + (skew(b2)*b_x) * (skew(r2)*r_x)') + b_x*r_x';
