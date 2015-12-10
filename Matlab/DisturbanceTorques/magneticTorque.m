function [mtorque, mdipole] = magneticTorque(q, b_eci)
% calculate magnetic torque due to magnetic dipole created by onboard
% electronics; magnetic torque is expressed in satellite reference frame
% ****************************************************
% ASSUMPTIONS:
% magnetic moment vector is modelled as pseudorandom uniformly distributed
% number for each component given the maximum value and resolution
% ****************************************************

mdipolemax = 1e-2; % max absolute value of magnetic dipole vector [Am2]
res = 1e-4; % resolution of magnetic dipole value [Am2]
range = round(mdipolemax/res);
mdipole = res * (randi(2 * range + 1, 3, 1) - range - 1);

A_i2s = q2m(q);
b_sat = A_i2s * b_eci;

mtorque = skew(mdipole) * b_sat * 1e-9; % magnetic field in [nT], thus 1e-9