function MagneticFieldMeas = MtmModel(MagneticFieldTrue, whiteNoise)
% Generate magnetometer measurement based on discrete model
% White noise added to MTM measurement with sigmaW
% Random Scale Factors % Misalignments included
% Constant bias included

global MtmErrors step;

MagneticFieldMeas = (eye(3) - MtmErrors.sigmaSFM*reshape(whiteNoise, [3 3])) * MagneticFieldTrue + MtmErrors.biasConst + ...
     MtmErrors.sigmaW / sqrt(step) * whiteNoise(1:3)';

end