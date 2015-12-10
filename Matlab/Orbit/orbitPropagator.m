function [r_eci, v_eci] = orbitPropagator(t, orb_elements0)
% Calculating satellite position & velocity in ECI

global J2 deriv_raan R_z mi;

%% reading initial elements
a0 = orb_elements0(1);
e0 = orb_elements0(2);
incl0 = orb_elements0(3);
raan0 = orb_elements0(4);
arg_perig0 = orb_elements0(5);
M0 = orb_elements0(6);

%% propagate orbital elements
a_mean = a0; % mean semimajor axis
e = e0;
incl = incl0; % [rad]
n_mean = sqrt(mi / a_mean^3); % mean motion

deriv_arg_perig = 3/2 * J2 * (R_z / a_mean)^2 * (1 - e^2)^2 * (2 - 5 / 2 * sin(incl)^2) * n_mean;
deriv_M = (1 + 3/4 * J2 * (R_z / a_mean)^2 / (1 - e^2)^(-3/2) * (3 * cos(incl)^2 - 1)) * n_mean;

raan = raan0 + deriv_raan * t; % [rad]
arg_perig = arg_perig0 + deriv_arg_perig * t; % [rad]
M = M0 + deriv_M * t; % [rad]

%% Solving Kepler equation for eccentric anomaly using Newton-Raphson iteration
threshold = 1e-8;

if e > 0.05
    E0 = M + e * sin(M) / (1 - e * cos(M)) - 1/2 * (e * sin(M) / (1 - e * cos(M)))^3; % initialize eccentric anomaly using series expansion to 3rd order
else
    E0 = M; % initialize eccentric anomaly with substituting M for small eccentricity
end

fcn = M - (E0 - e * sin(E0));
E = E0;
if abs(fcn) > threshold
    while abs(fcn) > threshold
        deriv_fcn = -(1 - e * cos(E));
        delta_E = fcn / deriv_fcn;
        E = E - delta_E;
        fcn = M - (E - e * sin(E));
    end % end while
end % end if

ni = 2 * atan2(sqrt(1 + e) * tan(E/2), sqrt(1 - e)) * 180 / pi; % real anomaly in [deg]

orb_elements = [a_mean, e, 180/pi*incl, 180/pi*raan, 180/pi*arg_perig, ni];

%% Converting orbital elements to perifocal coordinates [x, y]
n_true = deriv_M; % true mean motion including J2 and change of M0
a_true = (mi / n_true^2)^(1/3);

r_prf = a_true * (1 - e * cos(E)); % radius in perifocal frame
x_prf = a_true * (cos(E) - e); % position X in perifocal frame [km]
y_prf = a_true * sqrt(1 - e^2) * sin(E); % position Y in perifocal frame [km]

velx_prf = -n_true * a_true^2 * sin(E) / r_prf; % velocity X in perifocal frame [km/s]
vely_prf = n_true * a_true^2 * cos(E) * sqrt(1 - e^2) / r_prf; % velocity Y in perifocal frame [km/s]

%% conversion from perifocal to ECI
A_p2i = [cos(raan) * cos(arg_perig) - sin(raan) * sin(arg_perig) * cos(incl), -cos(raan) * sin(arg_perig) - sin(raan) * cos(arg_perig) * cos(incl); % first row
         sin(raan) * cos(arg_perig) + cos(raan) * sin(arg_perig) * cos(incl), -sin(raan) * sin(arg_perig) + cos(raan) * cos(arg_perig) * cos(incl); % second row
         sin(arg_perig) * sin(incl), cos(arg_perig) * sin(incl)]; % third row

r_eci = A_p2i * [x_prf, y_prf]'; % position column vector in ECI [x, y, z] [km]
v_eci = A_p2i * [velx_prf, vely_prf]'; % velocity column vector in ECI [vx, vy, vz] [km/s]

end