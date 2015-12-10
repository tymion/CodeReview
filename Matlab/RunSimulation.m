clear
clc

addpath(genpath('d:/PW-Sat2/ADCS/Matlab'))

% Read Simulation Parameters
simulationParameters();

global mode;

if strcmp(mode, 'sunPointing')
    [Output, T] = RunSunPointing();
    PlotSunPointingData(Output, T);
elseif strcmp(mode, 'detumbling')
    [Output, T] = RunDetumbling();
    PlotDetumblingData(Output, T);
end

