% Test funkcji sunPointing()

% dodaje sciezke do wszystkich folderow symulacji bo 
% funkcja sunPointing korzysta z funkcji w folderze Mathematics
addpath(genpath('D:/PW-Sat2/ADCS/Matlab')) 

clc;
clear;

angrate_s = [0.1 1 -2]';
b_sEst = [1 2 3]';
sun_vec_meas = [1 0 0]';

% definicja zmiennych globalnych
global Inertia_ref ctrlgains;
Inertia_ref = [1 2 3; 4 5 6; 7 8 9];
ctrlgains.k = 4e-3;
ctrlgains.kp = 4e-3;
ctrlgains.kn = -1e-4;
ctrlgains.angrate = [0.1 0 0]';

wynik = sunPointing(angrate_s, b_sEst, sun_vec_meas);