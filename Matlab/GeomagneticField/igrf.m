function [b_eci, b_eciLower] = igrf(t, r_eci, coeffs)
% Calculate geomagnetic field vector B in ECI coordinates given satellite
% position in ECI, time of simulation t and IGRF11 coefficients
% Lower order stands for reference (model) magnetic field:
% Assumed that True Magnetic Field is given by igrf_order and Reference
% Model is given by igrf_order - 1. In that way differences between True
% and Reference Fields are taken into account.

global date igrf_order;

%% transform position vector from ECI to ECEF

% Julian Date for hour, min, sec = 0  
jd = 1721013.5 + 367 * date.year - floor(7/4 * (date.year + floor((date.month + 9) / 12))) + floor(275 * date.month / 9) + date.day;
jd = 2458678.5;

% Number of Julian centuries from epoch J2000
T_uti = (jd - 2451545) / 36525;

% rotation angle around earth z axis from epoch J2000
theta = 24110.54841 + 8640184.812866 * T_uti + 0.093104 * T_uti^2 - 6.2 * 10^-6 * T_uti^3 + 1.002737909350795 * (3600 * date.hour + 60 * date.min + date.sec + t);
theta = theta - floor(theta / 86400) * 86400; % converting to the range <0;86400> [s]
theta = theta * pi / 43200; % converting to [rad]

A_i2e = [cos(theta),  sin(theta), 0;
         -sin(theta), cos(theta), 0;
         0,           0,          1]; % rotation matrix from eci to ecef
     
r_ecef = A_i2e * r_eci;

r = vectorNorm(r_ecef); % radial distance from earth center
colat = acos(r_ecef(3) / r); % colatitude = 90 - latitude
long = atan2(r_ecef(2), r_ecef(1)); % longitude

%% transform time of simulation to years units elapsed from latest igrf data
year0 = 2010; % year for newest igrf data generation
jd0 = 1721013.5 + 367*year0 - floor(7/4 * (year0 + floor((1 + 9)/12))) + floor(275 * 1/9) + 1;

jd = jd + (60 * date.hour + date.min + (date.sec + t) / 60) / 1440;

time = (jd - jd0) * 100 / 36525; % time elapsed from epoch 2010.0 to simulation time in [years]

%% Geomagnetic Field vector calculation in [north, east, down]
a = 6371.2; % magnetic reference spherical radius [km]

x = 0;
y = 0;
z = 0;

index = 1;

% Lower order IGRF - Reference IGRF simulated
for n = 1:igrf_order-1    
    for m = 0:n
        
        g_nm = coeffs(index, 1) + time * coeffs(index, 2);
        h_nm = coeffs(index, 3) + time * coeffs(index, 4);
                
        xlgndr = cos(colat);
        x_nm = (a/r)^(n + 1) * (g_nm * cos(m * long) + h_nm * sin(m * long)) * derivLegendre(n,m,xlgndr);
        y_nm = (a/r)^(n + 1) * (-g_nm * sin(m * long) * m + h_nm * cos(m * long) * m) * schLegendre(n,m,xlgndr);
        z_nm = (n + 1) * (a/r)^n * (-a / r^2) * (g_nm * cos(m * long) + h_nm * sin(m * long)) * schLegendre(n,m,xlgndr);
        
        x = x + x_nm;
        y = y + y_nm;
        z = z + z_nm;
        
        index = index + 1;        
    end % end m
end % end n

xLow = -x * sin(colat) * a / r;
yLow = -y * a / r / sin(colat);
zLow = z * a;
b_nedLower = [xLow, yLow, zLow]';

% True Magnetic Model calculated adding one more 'n' wrt reference model
n = igrf_order;
for m = 0:n
    g_nm = coeffs(index, 1) + time * coeffs(index, 2);
        h_nm = coeffs(index, 3) + time * coeffs(index, 4);
                
        index
        
        xlgndr = cos(colat);
        x_nm = (a/r)^(n + 1) * (g_nm * cos(m * long) + h_nm * sin(m * long)) * derivLegendre(n,m,xlgndr);
        y_nm = (a/r)^(n + 1) * (-g_nm * sin(m * long) * m + h_nm * cos(m * long) * m) * schLegendre(n,m,xlgndr);
        z_nm = (n + 1) * (a/r)^n * (-a / r^2) * (g_nm * cos(m * long) + h_nm * sin(m * long)) * schLegendre(n,m,xlgndr);
        
        x = x + x_nm;
        y = y + y_nm;
        z = z + z_nm;
        
        index = index + 1;
end % end m

xTrue = -x * sin(colat) * a / r;
yTrue = -y * a / r / sin(colat);
zTrue = z * a;
b_nedTrue = [xTrue, yTrue, zTrue]';

%% transform geomagnetic field vector from [north, east, down] to ECEF and then to ECI
lat = pi/2 - colat;
A_e2n = [-sin(lat)*cos(long), -sin(lat)*sin(long), cos(lat);
         -sin(long),          cos(long),           0;
         -cos(lat)*cos(long), -cos(lat)*sin(long), -sin(lat)]; % rotation matrix from ecef to ned
     
test = A_i2e' * A_e2n';
b_eci = A_i2e' * A_e2n' * b_nedTrue;
b_eciLower = A_i2e' * A_e2n' * b_nedLower; 

b_eci
b_eciLower

end