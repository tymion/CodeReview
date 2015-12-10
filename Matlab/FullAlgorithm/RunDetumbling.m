function [Output, T] = RunDetumbling()

%% Read IGRF coefficients & division of faces of the satellite
coeffs = getigrfcoeffs();
face = faceDiv();

global simtime orb_elements0 step norbits;

if rem(simtime, step) == 0
    T = simtime/norbits;
else
    T = (simtime - rem(simtime, step))/norbits;
end

%% Specify size of results matrices
size = floor(simtime/step) + 1;

%% Preallocate result matrices
Output.orbitPar = zeros(size,7);
Output.magneticField = zeros(size,4);
Output.eclipse = zeros(size,2);
Output.trueStateVector = zeros(size,7);
Output.torques = zeros(size,6);
Output.magneticDipole = zeros(size,3);
Output.dipoleApllied = zeros(size,4);
Output.BdotRaw = zeros(size,4);
Output.angRateNorm = zeros(size,2);
Output.power = zeros(size,3);

%% Read Initial quaternion & angular rates in SBRF
global q0_i2s angrate0 torque ctrlgains;

stateVector0 = [q0_i2s', angrate0'];

%% Generate noises
whiteNoise = randn(size+1,9);

%% Initialize energy & power consumption
energyCoils = 0;
powerCoilsprev = 0;

%% Initialize Bdot
% Position & velocity of the satellite in ECI
[r_eci, ~] = orbitPropagator(0, orb_elements0);

% Geomagnetic Field Vector in ECI
[b_eciTrue, ~] = igrf(0, r_eci, coeffs);

whiteNoise0 = whiteNoise(1,:); % extract noise vectors
magneticFieldMeasPrev = MtmModel(q2m(q0_i2s)*b_eciTrue, whiteNoise0); % in [nT]
magneticFieldMeasDerivPrev = 0;

%% Run Simulation
for t = 0:step:round(simtime)
    
    index = round(t/step);
    
    % Extract true quaternion and true angular rates from state vector
    qTrue = stateVector0(1:4)';
    angrateTrue = stateVector0(5:7)';
    
    % Position & velocity of the satellite in ECI 
    [r_eci, v_eci] = orbitPropagator(t, orb_elements0);
    
    % Geomagnetic Field Vector in ECI
    [b_eciTrue, ~] = igrf(t, r_eci, coeffs);
        
    % Earth 2 Sun unit vector in ECI
    r_e2s = earth2sunVector(t);
    
    % Check if eclipse 
    check = - sqrt(vectorNorm(r_eci)^2 - (6378.137 + 20)^2); % extended equatorial radius wgs84 [km]
    if r_eci' * r_e2s < check
        eclipse = 1;
    else
        eclipse = 0;
    end
    
    % Disturbance torques
    aeroTorque = aeroDragTorque(r_eci, v_eci, qTrue, face);
    ggTorque = gravGrad(qTrue, r_eci, v_eci);
    sunTorque = solarTorque(r_e2s, qTrue, eclipse, face);
    [magTorque, mdipoleDist] = magneticTorque(qTrue, b_eciTrue);
    distTorque = ggTorque + sunTorque + aeroTorque + magTorque;
    
    % Generate measurements
    whiteNoise_i = whiteNoise(index+2,:); % extract noise vectors
    magneticFieldMeas = MtmModel(q2m(qTrue)*b_eciTrue, whiteNoise_i); % in [nT]
    magneticFieldMeasDeriv = exp(-ctrlgains.cut_off * step) * magneticFieldMeasDerivPrev + ctrlgains.cut_off * ...
        (magneticFieldMeas - magneticFieldMeasPrev); % in [nT/s]
    magneticFieldMeasPrev = magneticFieldMeas;
    magneticFieldMeasDerivPrev = magneticFieldMeasDeriv;
        
    % Control torque
    commDipole = Detumbling(magneticFieldMeas, magneticFieldMeasDeriv);
    [ctrlTorque, dipoleApplied, powerCoils] = magnetorquers(b_eciTrue, qTrue, commDipole);
    energyCoils = energyCoils + step*powerCoilsprev;
    powerCoilsprev = powerCoils;
    
    % Integrate true angular velocity and true attitude quaternion
    torque = distTorque + ctrlTorque;
    timerange = [t, t + step];
    [~, stateVector_i] = ode45(@Integrate, timerange, stateVector0);
    stateVector_i(end,1:4) = qnorm(stateVector_i(end,1:4)); % normalization of quaternion
    stateVector0 = stateVector_i(end,:); % initial stateVector in next iteration
    
    % Create output matrices     
    Output.orbitPar(index+1,:) = [t, r_eci', v_eci'];
    Output.magneticField(index+1,:) = [t, b_eciTrue'];
    Output.trueStateVector(index+1,:) = [t, q2euler(qTrue)', 180/pi*angrateTrue'];
    Output.eclipse(index+1,:) = [t, eclipse];
    Output.torques(index+1,:) = [t, vectorNorm(aeroTorque), vectorNorm(ggTorque), ...
        vectorNorm(sunTorque), vectorNorm(magTorque), vectorNorm(ctrlTorque)];
    Output.magneticDipole(index+1,:) = [t, vectorNorm(mdipoleDist), vectorNorm(dipoleApplied)];
    Output.dipoleApllied(index+1,:) = [t, dipoleApplied'];
    Output.BdotRaw(index+1,:) = [t, magneticFieldMeasDeriv'];
    Output.angRateNorm(index+1,:) = [t, vectorNorm(Output.trueStateVector(index+1,5:7))];
    Output.power(index+1,:) = [t, powerCoils, energyCoils/3600]; % conversion of energy from [J] to [Wh]   
                
end

end
