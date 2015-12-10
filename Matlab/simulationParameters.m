function simulationParameters()

%% Path to data folder
global files_path;
files_path = 'd:/PW-Sat2/ADCS/Matlab';

%% Time constants

% ***************************************************** %
%   Initial year month day hour minute and second
%   this is t0 for simulation
% ***************************************************** %

global date simtime step norbits;

date.year = 2015;
date.month = 3;
date.day = 20;
date.hour = 17;
date.min = 0;
date.sec = 0;

norbits = 2; % specify number of orbits for simulation
step = 0.2; % time step for simulation [s] : DEPENDS ON CONTROL MODE, ~0.2s for detumbling, ~1s for SP

%% SATELLITE 
% ********************************* %

global Inertia r_cm Inertia_ref;

Ixx = 0.012356; 
Iyy = 0.011097; 
Izz = 0.004432;

Ixy = 0.000016;
Ixz = -0.000016; 
Iyz = 0.000042; 

Inertia = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz]; % [kg*m2]
Inertia_ref = 0.8 * Inertia;
r_cm = [0.005 0.004 -0.003]'; % vector from satellite geometrical origin to the center of mass expressed in satellite frame

%% SSO ORBIT

% **************************************** %
%   simulation starts from perigee
%   earth as a sphere -> radius const
%   only J2 included in perturbation model
%
%   semimajor axis, inclination 
%   and eccentricity constant
% **************************************** %

global J2 deriv_raan R_z mi orb_elements0;

R_z = 6378.137; % [km] earth equator radius from WGS84
M_z = 5.9736e24; % [kg] earth mass
G = 6.6738e-20; % gravity constant
mi = G * M_z;

h0 = 600; % initial orbit altitude in perigee [km]
J2 = 1.08262668355e-3; % second zonal harmonic coefficient
deriv_raan = 1.99106e-7; % raan derivative for SSO
coeff = -3 / 2 * J2 * R_z^2 * sqrt(mi) / deriv_raan; % coefficient for determining inclination for SSO

a0 = R_z + h0; % initial semimajor axis [km]
e0 = 0; % orbit eccentricity [-]
incl0 = acos(1 / coeff * (1 - e0^2)^2 * a0^(7/2)); % inclination calculated for SSO based on a0 and ecc [rad]
raan0 = 0 * pi / 180; % initial raan [rad]
arg_perig0 = 0 * pi / 180; % initial argument of perigee [rad]
M0 = 0 * pi /180; % initial mean anomaly [rad]

% Creating initial orbital elements vector
orb_elements0 = [a0, e0, incl0, raan0, arg_perig0, M0]'; 

% Converting number of orbits to seconds
n_true = (1 + 3/4 * J2 * (R_z / a0)^2 / (1 - e0^2)^(-3/2) * (3 * cos(incl0)^2 - 1)) * sqrt(mi / a0^3);
a_true = (mi / n_true^2)^(1/3);
T = 2 * pi / sqrt(mi) * (a_true)^(3/2);
simtime = floor(norbits * T);

%% INITIAL ATTITUDE AND RATE
% ********************************* %

global angrate0 q0_i2s;

% initial euler angles of satellite frame wrt eci frame
yaw0 = 75 * pi / 180;
pitch0 = 10 * pi / 180;
roll0 = -25 * pi / 180;
p0 = 10 * pi / 180;
q0 = 10 * pi / 180;
r0 = 10 * pi / 180;
angrate0 = [p0 q0 r0]'; % [rad/s] in satellite frame

% Initial quaternion
q0_1 = cos(yaw0/2) * cos(pitch0/2) * sin(roll0/2) - sin(yaw0/2) * sin(pitch0/2) * cos(roll0/2);
q0_2 = cos(yaw0/2) * sin(pitch0/2) * cos(roll0/2) + sin(yaw0/2) * cos(pitch0/2) * sin(roll0/2);
q0_3 = sin(yaw0/2) * cos(pitch0/2) * cos(roll0/2) - cos(yaw0/2) * sin(pitch0/2) * sin(roll0/2);
q0_4 = cos(yaw0/2) * cos(pitch0/2) * cos(roll0/2) + sin(yaw0/2) * sin(pitch0/2) * sin(roll0/2);
q0_i2s = [q0_1 q0_2 q0_3 q0_4]';
q0_i2s = qnorm(q0_i2s);

%% CONTROL MODE
% *******************************************
% Type 'detumbling' for detumbling mode with b-dot algorithm
% Type 'sunPointing' for Sun pointing with spin stabilization
% *******************************************

global mode ctrlgains;

mode = 'detumbling';

if strcmp(mode, 'sunPointing')
    % commanded angular values in satellite frame
    p_comm = 5 * pi / 180;
    q_comm = 0;
    r_comm = 0;
    ctrlgains.angrate = [p_comm, q_comm, r_comm]';
    
    % angular momentum, precession and nutation gains
    ctrlgains.k = 4e-3;
    ctrlgains.kp = 4e-3;
    ctrlgains.kn = -1e-4;
elseif strcmp(mode, 'detumbling')
    ctrlgains.bdot = 6*pi/T * (1 + sin(incl0 - 10*pi/180)) * Izz;
    ctrlgains.cut_off = 0.2;
else
    error('Unspecified control mode. Only Detumbling and Sun Pointing available.')
end

%% MAGNETORQUERS PARAMETERS
% *********************************************

global selectcoils maxdipole_xy maxdipole_z freq power_xy power_z;

selectcoils = diag([1 1 1]); % (x y z) type 1 for active, 0 for nonactive
maxdipole_xy = 0.2; % [Am2]
maxdipole_z = 0.24; % [Am2]
power_xy = 1.1; % for rods [W/Am2]
power_z = 2.9; % for air coil [W/Am2]

timeon = 0.8; % [s]
timeoff = 0.2; % [s]
freq = timeon / (timeon+timeoff); % [-]

%% IGRF11 MODEL
% *********************************************

global igrf_order;

igrf_order = 10; % order of harmonic potential expansion for TRUE model

%% SENSORS
% *********************************************
% GYRO:
% constant bias, drift, white noise, SF & MA included
%
% MAGNETOMETER:
% constant bias, white noise, SF & MA included
%
% SUN SENSOR:
% constant bias, white noise, SF & MA included
% photodiodes treated as a system

global GyroErrors MtmErrors SunsErrors;

GyroErrors.sigmaB = 0.005 * pi/180; % bias instability in [rad/s/sqrt(s)]
GyroErrors.sigmaW = 0.5 * pi/180; % noise in [rad/s*sqrt(s)]
GyroErrors.biasConst = [0.1, 0.2, -0.1]' * pi/180; % constant bias [rad/s]
GyroErrors.sigmaSFM = 0.02; % rms of scale factors & misalignments errors

MtmErrors.sigmaW = 150; % noise in [nT]
MtmErrors.biasConst = [800, 700, -650]'; % constant bias [nT]
MtmErrors.sigmaSFM = 0.02; % rms of scale factors & misalignments errors

SunsErrors.sigmaW = sin(6*pi/180); % rms deviation between true & measured unit vectors
SunsErrors.biasConst = [0.02, -0.02, 0.03]'; % constant bias of unit vector components
SunsErrors.sigmaSFM = 0.02; % rms of scale factors & misalignments errors


%% EXTENDED KALMAN FILTER
% **********************************************

global KalmanCovariance sigmaWahba;

sigmaWahba.r(1) = sin(0.5*pi/180);
sigmaWahba.b(1) = sin(3*pi/180);
sigmaWahba.r(2) = sin(0.3*pi/180);
sigmaWahba.b(2) = sin(6*pi/180);

KalmanCovariance.R = 0.9 * diag([sin(3*pi/180)^2, sin(3*pi/180)^2, sin(3*pi/180)^2, ...
    sin(6*pi/180)^2, sin(6*pi/180)^2, sin(6*pi/180)^2, ...
    GyroErrors.sigmaW^2, GyroErrors.sigmaW^2, GyroErrors.sigmaW^2]); 

KalmanCovariance.Q = diag([1 1 1 10 10 10])*1e-8;
KalmanCovariance.P0 = diag([1 1 1 10 10 10])*1e-4;


