function [angrateMeas, bias] = GyroModel(angrateTrue, biasTruePrev, whiteNoise)
% Generate gyroscope measurement based on discrete gyro model
% Bias modeled as random walk process with sigmaB
% White noise added to gyro measurement with sigmaW
% Random Scale Factors % Misalignments included
% Constant bias included

global GyroErrors step;

bias = biasTruePrev + GyroErrors.sigmaB*sqrt(step) * flipud(whiteNoise(1:3)');
angrateMeas = (eye(3) - GyroErrors.sigmaSFM*reshape(whiteNoise, [3 3])') * angrateTrue + GyroErrors.biasConst + ...
    0.5*(bias + biasTruePrev) + sqrt(GyroErrors.sigmaW^2 / step + GyroErrors.sigmaB^2 * step/12) * whiteNoise(7:9)';
    
end