function SunSensorMeas = SunsModel(SunVectorTrue, eclipse, whiteNoise)
% Generate sun sensor measurement based on discrete model
% White noise added to SunS measurement with sigmaW
% Random Scale Factors % Misalignments included
% Constant bias included

global SunsErrors step;

if eclipse == 1
    SunSensorMeas = [0 0 0]';
    return
end

SunSensorMeas = (eye(3) - SunsErrors.sigmaSFM*flipud(reshape(whiteNoise, [3 3]))) * SunVectorTrue + SunsErrors.biasConst + SunsErrors.sigmaW / sqrt(step) * whiteNoise(4:6)';

SunSensorMeas = SunSensorMeas / vectorNorm(SunSensorMeas); 
 
end