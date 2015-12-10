clear
clc

addpath(genpath('c:/users/pawel/desktop/adcs symulacja pw-sat2/matlab'))

global SunsErrors step;
SunsErrors.sigmaW = sin(3.5*pi/180); % rms deviation between true & measured unit vectors
SunsErrors.biasConst = [0.015, 0.01, -0.02]'; % constant bias of unit vector components
SunsErrors.sigmaSFM = 0.02; % rms of scale factors & misalignments errors

% Initialize
step = 1;

for i = 1:20000
    
    SunVectorTrue = [i/20, 400, 350]';
    SunVectorTrue = SunVectorTrue / vectorNorm(SunVectorTrue);
    SunVectorMeas = SunsModel(SunVectorTrue, 0, 0);
    error = acos(SunVectorTrue' * SunVectorMeas) * 180/pi;
    
    results(i,:) = [i, SunVectorMeas', error];
end

figure;
plot(results(:,1), results(:,2), 'b-', 'LineWidth', 1)
hold on
grid on
plot(results(:,1), results(:,3), 'r-', 'LineWidth', 1)
hold on
plot(results(:,1), results(:,4), 'g-', 'LineWidth', 1)
title('SunS Output')
legend('X', 'Y', 'Z')
xlabel('No. of iteration [-]')

figure;
plot(results(:,1), results(:,5), 'b-', 'LineWidth', 1)
grid on
title('SunS Error between Measurement & True [deg]')
xlabel('No. of iteration [-]')