function gg_torque = gravGrad(q, r_eci, v_eci)
% Calculating gravity gradient torque in SBRF frame

global mi Inertia;

r = vectorNorm(r_eci);
A_i2s = q2m(q);  

g3 = 1 / vectorNorm(r_eci);
g2 = 1 / vectorNorm(skew(r_eci) * v_eci);

oz_i = -g3 * r_eci; % unit vector of orbital frame z axis expressed in eci
oy_i = -g2 * skew(r_eci) * v_eci; % unit vector of orbital frame y axis expressed in eci
ox_i = g2 * g3 * (vectorNorm(r_eci)^2 * v_eci - r_eci' * v_eci * r_eci); % unit vector of orbital frame x axis expressed in eci
A_o2i = [ox_i, oy_i, oz_i]; % rotation matrix from orbital to eci frame

A_o2s = A_i2s * A_o2i;

gg_x = 3 * mi/r^3 * (Inertia(3,3) - Inertia(2,2)) * A_o2s(2,3) * A_o2s(3,3);
gg_y = 3 * mi/r^3 * (Inertia(1,1) - Inertia(3,3)) * A_o2s(1,3) * A_o2s(3,3);
gg_z = 3 * mi/r^3 * (Inertia(2,2) - Inertia(1,1)) * A_o2s(1,3) * A_o2s(2,3);

gg_torque = [gg_x gg_y gg_z]';

end