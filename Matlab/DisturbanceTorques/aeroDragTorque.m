function aero_torque = aeroDragTorque(r_eci, v_eci, q, face)
% Calculate aerodynamic drag torque in satellite frame
% ************************************************
% ASSUMPTIONS:
% 1. atmosphere corotates with earth;
% 2. geocentric latitude calculated for distance to satellite, not to
% intersection point with ellipsoid;
% 3. model accounts for change of earth radius with latitude;
% 4. density calculated only in 450 - 800 km altitude;
% 5. exponentially decaying static model of the atmosphere
% 6. torque calculated using division of the satellite into separate faces
% ************************************************

global R_z;

w_ez = 7.2921158553e-5; % [rad/s]
w_e = w_ez * [0 0 1]'; % earth rate vector in eci
vi_rel = v_eci - skew(w_e) * r_eci; % relative satellite velocity to the atmosphere in eci [km/s]
A_i2s = q2m(q);
vs_rel = A_i2s * vi_rel; % relative velocity in satellite frame [km/s]

alt = vectorNorm(r_eci) - R_z; % approx altitude above spherical earth [km]

if alt >= 450 && alt < 500
    ro0 = 1.585e-12;
    H = 62.2;
    h0 = 450;
elseif alt >= 500 && alt < 600
    ro0 = 6.967e-13;
    H = 65.8;
    h0 = 500;
elseif alt >= 600 && alt < 700
    ro0 = 1.454e-13;
    H = 79;
    h0 = 600;
elseif alt >= 700 && alt <= 800
    ro0 = 3.614e-14;
    H = 109;
    h0 = 700;
else
    error('Altitude only in the range 450 - 800 km')
end

ro = ro0 * exp((h0 - alt) / H);
cd = 2.2; % drag coeff

aero_torque = 0;
for i = 1:length(face.s)
    cos_i = face.v(:,i)' * vs_rel / vectorNorm(vs_rel);
    force_i = - 0.5 * ro * cd * vectorNorm(vs_rel) * vs_rel * face.s(i) * max(cos_i, 0) * 1e6; % [N]
    aero_torque_i = skew(face.cp(:,i)) * force_i;
    
    aero_torque = aero_torque + aero_torque_i;
end

end