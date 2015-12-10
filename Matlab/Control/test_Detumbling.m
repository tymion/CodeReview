% Test funkcji Detumbling()

% dodaje sciezke do wszystkich folderow symulacji bo 
% funkcja sunPointing korzysta z funkcji w folderze Mathematics
addpath(genpath('D:/PW-Sat2/ADCS/Matlab')) 

bMeas = [1 2 3]';
bDeriv = [-2 0.1 -1]';

% definicja zmiennych globalnych
global ctrlgains;
ctrlgains.bdot = 2;

wynik = Detumbling(bMeas, bDeriv)