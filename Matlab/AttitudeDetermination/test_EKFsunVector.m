% Test funkcji EKFsunVector()
clear;
clc;

% dodaje sciezke do wszystkich folderow symulacji bo 
% funkcja sunPointing korzysta z funkcji w folderze Mathematics
addpath(genpath('d:/pw-sat2/adcs/matlab')) 

b_eci = [1 2 3]';
r_e2s_eci = [0 2 -1]';
r_e2s_eci = r_e2s_eci / vectorNorm(r_e2s_eci); % normalizacja
time_step = 1;
eclipse = 0;
P_k = [1 0 0 0 0 0;
    0 2 0 0 0 0;
    0 0 6 0 0 0;
    0 0 0 2 0 0;
    0 0 0 0 3 0;
    0 0 0 0 0 9]*1e-6;
Z = [1; 2; 3; -1; 2; 0.5; 0.1; 0.2; -0.1];
v_rot = [1 2 3]';
v_rot = v_rot / vectorNorm(v_rot); % normalizacja
q = [v_rot*sin(60*pi/180); cos(60*pi/180)];
X_k = [q; 0.11; 0.22; -0.15];

% definicja zmiennych globalnych
global KalmanCovariance Inertia_ref ctrlTorque_prev;
KalmanCovariance.R = [1 0 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0;
    0 0 0 2 0 0 0 0 0;
    0 0 0 0 2 0 0 0 0;
    0 0 0 0 0 2 0 0 0;
    0 0 0 0 0 0 4 0 0;
    0 0 0 0 0 0 0 4 0;
    0 0 0 0 0 0 0 0 4];

KalmanCovariance.Q = [1e-8 0 0 0 0 0;
    0 1e-8 0 0 0 0;
    0 0 1e-8 0 0 0;
    0 0 0 1e-7 0 0;
    0 0 0 0 1e-7 0;
    0 0 0 0 0 1e-7];

Inertia_ref = [1 2 3; 2 5 1; 3 1 9]*1e-4;
ctrlTorque_prev = [1 2 3]'*1e-8;

[wynik1, wynik2, wynik3, wynik4] = EKFsunVector(X_k, P_k, Z, eclipse, b_eci, r_e2s_eci, time_step);