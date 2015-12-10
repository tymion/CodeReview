% Test funkcji igrf

% dodaje sciezke do wszystkich folderow symulacji bo 
% funkcja igrf korzysta z funkcji w folderze Mathematics
addpath(genpath('d:/pw-sat2/adcs/matlab')) 

t = 100;
r_eci = [6800, -120, 1000]';
coeffs = load('igrf12coeffs.txt');

% Definicja zmiennych globalnych w funkcji igrf()
global igrf_order date
igrf_order = 6;
date.year = 2015;
date.month = 5;
date.day = 24;
date.hour = 18;
date.min = 0;
date.sec = 0;

[wynik1, wynik2] = igrf(t, r_eci, coeffs);