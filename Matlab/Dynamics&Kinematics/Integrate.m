function deriv = Integrate(~, z)
% Right hand side function for RK4 procedure
% State Vector z = [q, angrate] where:
%  - q is attitude quaternion [q1, q2, q3, q4] where q4 is a scalar part
%    q represents attitude of SBRF wrt ECI
%  - angrate = [wx, wy, wz]
%    angrate is angular rate vector of SBRF wrt ECI expressed in SBRF

global Inertia torque;

q = qnorm(z(1:4));
angrate = z(5:7);

deriv_q = 0.5 * [-skew(angrate), angrate; -angrate', 0] * q;
deriv_angrate = Inertia \ (torque - skew(angrate)*(Inertia*angrate)); 

deriv = [deriv_q; deriv_angrate];

end