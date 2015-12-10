function commDipole = Detumbling(bMeas, bDeriv)
% Calculating desired control dipole for Detumbling mode [Am2]
% desired control torque is expressed in satellite frame
% control law is based on B-Dot algorithm
% underactuation of the coils is included

global ctrlgains;
k = ctrlgains.bdot;

commDipole = -k*bDeriv*1e-9 / vectorNorm(bMeas*1e-9)^2;