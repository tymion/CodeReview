% Test funkcji sunPointingRaw()

% dodaje sciezke do wszystkich folderow symulacji bo 
% funkcja sunPointing korzysta z funkcji w folderze Mathematics
addpath(genpath('D:/PW-Sat2/ADCS/Matlab')) 

clc;
clear;

angrate_s = [0.1 1 -2]';
b_sEst = [1 2 3]';
qEst = [0; sin(30*pi/180); 0; cos(30*pi/180)];

r_e2s = [4; 10; -5];
r_e2s = r_e2s / vectorNorm(r_e2s) % wektor jednostkowy

% definicja zmiennych globalnych
global Inertia_ref ctrlgains;
Inertia_ref = [1 2 3; 4 5 6; 7 8 9];
ctrlgains.k = 4e-3;
ctrlgains.kp = 4e-3;
ctrlgains.kn = -1e-4;
ctrlgains.angrate = [0.1 0 0]';

wynik = sunPointingKF(angrate_s, r_e2s, qEst, b_sEst)