% Test funkcji WahbaTriadWeight()

clear;
clc;

% dodaje sciezke do wszystkich folderow symulacji bo 
% funkcja sunPointing korzysta z funkcji w folderze Mathematics
addpath(genpath('d:/pw-sat2/adcs/matlab')) 

r1 = [0.1 1 -2]';
r1 = r1 / vectorNorm(r1); % normalizacja
r2 = [5 1 -0.2]';
r2 = r2 / vectorNorm(r2); % normalizacja

v_rot = [1 2 3]';
v_rot = v_rot / vectorNorm(v_rot); % normalizacja
q = [v_rot*sin(60*pi/180); cos(60*pi/180)];
A_r2b = q2m(q);

b1 = A_r2b * r1;
b2 = A_r2b * r2;

% definicja zmiennych globalnych
global sigmaWahba;
sigmaWahba.r(1) = sin(0.1*pi/180);
sigmaWahba.b(1) = sin(3*pi/180);
sigmaWahba.r(2) = sin(0.3*pi/180);
sigmaWahba.b(2) = sin(6*pi/180);

wynik = WahbaTriadWeight(b1, b2, r1, r2)

% ***************************************
% UWAGA: macierz 'wynik' powinna byc rowna macierzy A_r2b
% ***************************************