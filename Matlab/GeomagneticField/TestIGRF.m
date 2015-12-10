% ****************** VALIDATION OF IGRF() FUNCTION ***********************
% Code tests the results for given GEODETIC latitude, longitude,
% altitude and date.
% Result is Magnetic Field vector based on IGRF 11 Model. Output vector is
% expressed in GEODETIC North East Down. Function igrf.m in simulation
% calculates Magnetic Field vector in GEOCENTRIC North East Down and then
% convert to ECEF and ECI. Here output is in geodetic coordinates in order
% to easily compare data with Online IGRF Calculator.

clc
clear

addpath(genpath('c:/users/pawel/desktop/adcs symulacja pw-sat2/matlab'))

%% Specify geodetic latitude, longitude and altitude
lat = 80 * pi / 180;
longitude = 70 * pi / 180;
h = 100;

%% Specify date
date.year = 2014;
date.month = 12;
date.day = 3;
date.hour = 12;
date.min = 0;
date.sec = 0;

%% WGS84 Parameters
r_e = 6378.137;		% equator radius [km]	
eps = 0.0818191908426;	% first eccentricity

%% Conversion to ECEF given geodetic coordinates
x_ecef = (r_e / sqrt(1 - eps^2 * sin(lat)^2) + h) * cos(lat) * cos(longitude);
y_ecef = (r_e / sqrt(1 - eps^2 * sin(lat)^2) + h) * cos(lat) * sin(longitude);
z_ecef = (r_e * (1 - eps^2) / sqrt(1 - eps^2 * sin(lat)^2) + h) * sin(lat);

%% Calculate geocentric latitude given geodetic and rotation matrix between geocentric & geodetic
lat_gc = atan(tan(lat) * (r_e * (1 - eps^2) / sqrt(1 - eps^2 * sin(lat)^2) + h) / (r_e / sqrt(1 - eps^2 * sin(lat)^2) + h));
delta_lat = lat - lat_gc;

A = [cos(delta_lat),  0, sin(delta_lat);
     0,               1, 0;
     -sin(delta_lat), 0, cos(delta_lat)];

%% Specify IGRF 11 order
igrf_order = 13;

%% Load IGRF 11 coefficients
coeffs = getigrfcoeffs();

%% **************** HERE ALGORITHM OF IGRF() FUNCTION STARTS *******************

jd = 1721013.5 + 367 * date.year - floor(7/4 * (date.year + floor((date.month + 9) / 12))) + floor(275 * date.month / 9) + date.day;
     
r_ecef = [x_ecef, y_ecef, z_ecef]';

r = vectorNorm(r_ecef); % radial distance from earth center
colat = acos(r_ecef(3) / r); % colatitude = 90 - latitude
long = atan2(r_ecef(2), r_ecef(1)); % longitude

% transform time of simulation to years units elapsed from latest igrf data
year0 = 2010; % year for newest igrf data generation
jd0 = 1721013.5 + 367*year0 - floor(7/4 * (year0 + floor((1 + 9)/12))) + floor(275 * 1/9) + 1;

jd = jd + (60 * date.hour + date.min + date.sec / 60) / 1440;

time = (jd - jd0) * 100 / 36525; % time elapsed from epoch 2010.0 to simulation time in [years]

% Geomagnetic Field vector calculation in [north, east, down]
a = 6371.2; % magnetic reference spherical radius [km]

x = 0;
y = 0;
z = 0;

index = 1;

for n = 1:igrf_order
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

x = -x * sin(colat) * a / r;
y = -y * a / r / sin(colat);
z = z * a;
b_ned = [x, y, z]';
b_ned = A * b_ned;
norm = vectorNorm(b_ned);

