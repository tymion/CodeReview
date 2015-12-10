function [Output, T] = RunSunPointing()

%% Read IGRF coefficients & division of faces of the satellite
coeffs = getigrfcoeffs();
face = faceDiv();

global simtime orb_elements0 step norbits ctrlgains;

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
Output.magneticFieldComp = zeros(size,4);
Output.eclipse = zeros(size,2);
Output.trueStateVector = zeros(size,7);
Output.torques = zeros(size,6);
Output.magneticDipole = zeros(size,3);
Output.dipoleApllied = zeros(size,4);
Output.omegaCtrlError = zeros(size,4);
Output.angleError = zeros(size,3);
Output.estimate = zeros(size,7);
Output.omegaNavError = zeros(size,3);
Output.attNavError = zeros(size,3);
Output.power = zeros(size,3);

%% Read Initial quaternion & angular rates in SBRF
global q0_i2s angrate0 torque KalmanCovariance ctrlTorque_prev;

stateVector0 = [q0_i2s', angrate0'];
ctrlTorque_prev = 0;

%% Generate noises
whiteNoise = randn(size+1,9);

%% Initialize Kalman filter with TRIAD
% Extract true quaternion and true angular rates from state vector
qTrue = stateVector0(1:4)';
angrateTrue = stateVector0(5:7)';

% Position & velocity of the satellite in ECI
[r_eci, ~] = orbitPropagator(0, orb_elements0);

% Geomagnetic Field Vector in ECI
[b_eciTrue, b_eciRef] = igrf(0, r_eci, coeffs);

% Earth 2 Sun unit vector in ECI
r_e2s = earth2sunVector(0);

% Check if eclipse
check = - sqrt(vectorNorm(r_eci)^2 - (6378.137 + 20)^2); % extended equatorial radius wgs84 [km]
if r_eci' * r_e2s < check
    error('Cannot solve Wahba Problem in Eclipse. Try another date.')
else
    eclipse = 0;
end

% Generate measurements
whiteNoise0 = whiteNoise(1,:);

magneticFieldMeas = MtmModel(q2m(qTrue)*b_eciTrue, whiteNoise0);
sunSensorMeas = SunsModel(q2m(qTrue)*r_e2s, eclipse, whiteNoise0);
[angrateMeas, bias_prev] = GyroModel(angrateTrue, 0, whiteNoise0);

magneticFieldMeasUnit = magneticFieldMeas / vectorNorm(magneticFieldMeas);

% Estimated quaternion, angular rates, magnetic field
A = WahbaTriadWeight(magneticFieldMeasUnit, sunSensorMeas, b_eciRef/vectorNorm(b_eciRef), r_e2s);
qEst = m2q(A);
qEst = qnorm(qEst);
angrateEst = angrateMeas;
Xprev = [qEst; angrateEst];
Pprev = KalmanCovariance.P0;

%% Initialize energy & power consumption
energyCoils = 0;
powerCoilsprev = 0;

%% Run Simulation
for t = 0:step:round(simtime)
    
    % Extract true quaternion and true angular rates from state vector
    qTrue = stateVector0(1:4)';
    angrateTrue = stateVector0(5:7)';
    
    % Position & velocity of the satellite in ECI 
    [r_eci, v_eci] = orbitPropagator(t, orb_elements0);
    
    % Geomagnetic Field Vector in ECI
    [b_eciTrue, b_eciRef] = igrf(t, r_eci, coeffs);
        
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
    whiteNoise_i = whiteNoise(t/step+2,:); % extract noise vectors
    
    magneticFieldMeas = MtmModel(q2m(qTrue)*b_eciTrue, whiteNoise_i);
    sunSensorMeas = SunsModel(q2m(qTrue)*r_e2s, eclipse, whiteNoise_i);
    [angrateMeas, bias] = GyroModel(angrateTrue, bias_prev, whiteNoise_i);
    bias_prev = bias;
        
    Z = [magneticFieldMeas', sunSensorMeas', angrateMeas']';
    
    % Estimated quaternion, angular rates, magnetic field
    [X, P] = EKFnoBias(Xprev, Pprev, Z, eclipse, b_eciRef, r_e2s);
    qEst = X(1:4);
    angrateEst = X(5:7);
    Xprev = X;
    Pprev = P;
    
    b_sEst = magneticFieldMeas; % in [nT]
        
    % Control torque
    commDipole = sunPointing(angrateEst, r_e2s, qEst, b_sEst);
    [ctrlTorque, dipoleApplied, powerCoils] = magnetorquers(b_eciTrue, qTrue, commDipole);
    
    % TRIAD algorithm
    magneticFieldMeasUnit = magneticFieldMeas / vectorNorm(magneticFieldMeas);
    
    if eclipse == 0
        A = WahbaTriadWeight(magneticFieldMeasUnit, sunSensorMeas, b_eciRef/vectorNorm(b_eciRef), r_e2s);
        qTriad = m2q(A);
        qTriad = qnorm(qTriad);
        stateVectorTriad0 = [qTriad', angrateMeas'];
    else
        torque = 0;
        timerange = [t, t + step];
        [~, stateVectorTriad_i] = ode45(@Integrate, timerange, stateVectorTriad0);
        stateVectorTriad_i(end,1:4) = qnorm(stateVectorTriad_i(end,1:4)); % normalization of quaternion
        stateVectorTriad0 = stateVectorTriad_i(end,:); % initial stateVector in next iteration
        qTriad = stateVectorTriad_i(end,1:4)';
        
        ctrlTorque = [0, 0, 0]';
        dipoleApplied = [0, 0, 0]';
        powerCoils = 0;
    end
    
    ctrlTorque_prev = skew(dipoleApplied) * b_sEst * 1e-9;
    energyCoils = energyCoils + step*powerCoilsprev;
    powerCoilsprev = powerCoils;
    
    % Integrate true angular velocity and true attitude quaternion
    torque = distTorque + ctrlTorque;
    timerange = [t, t + step];
    [~, stateVector_i] = ode45(@Integrate, timerange, stateVector0);
    stateVector_i(end,1:4) = qnorm(stateVector_i(end,1:4)); % normalization of quaternion
    stateVector0 = stateVector_i(end,:); % initial stateVector in next iteration
    
    % Create output matrices   
    Output.orbitPar(t/step+1,:) = [t, r_eci', v_eci'];
    Output.magneticField(t/step+1,:) = [t, b_eciTrue'];
    Output.magneticFieldComp(t/step+1,:) = [t, (b_eciTrue - b_eciRef)'];
    Output.trueStateVector(t/step+1,:) = [t, q2euler(qTrue)', 180/pi*angrateTrue'];
    Output.eclipse(t/step+1,:) = [t, eclipse];
    Output.torques(t/step+1,:) = [t, vectorNorm(aeroTorque), vectorNorm(ggTorque), ...
        vectorNorm(sunTorque), vectorNorm(magTorque), vectorNorm(ctrlTorque)];
    Output.magneticDipole(t/step+1,:) = [t, vectorNorm(mdipoleDist), vectorNorm(dipoleApplied)];
    Output.dipoleApllied(t/step+1,:) = [t, dipoleApplied'];
    Output.omegaCtrlError(t/step+1,:) = [t, 180/pi*(ctrlgains.angrate - angrateTrue)'];
    Output.estimate(t/step+1,:) = [t, q2euler(qEst)', 180/pi*angrateEst'];
    
    Xerror = acos(r_e2s' * q2m(qTrue)' * [1 0 0]') * 180/pi;
    omegaerror = acos(r_e2s' * q2m(qTrue)' * angrateTrue / vectorNorm(angrateTrue)) * 180/pi;
    Output.angleError(t/step+1,:) = [t, Xerror, omegaerror];
    
    qErrorEKF = [qTrue(4)*eye(3)-skew(qTrue(1:3)), qTrue(1:3); -qTrue(1:3)', qTrue(4)] * [-qEst(1), -qEst(2), -qEst(3), qEst(4)]';
    qErrorTriad = [qTrue(4)*eye(3)-skew(qTrue(1:3)), qTrue(1:3); -qTrue(1:3)', qTrue(4)] * [-qTriad(1), -qTriad(2), -qTriad(3), qTriad(4)]';
    Output.attNavError(t/step+1,:) = [t, 180/pi*2*vectorNorm(qErrorEKF(1:3)), 180/pi*2*vectorNorm(qErrorTriad(1:3))];
            
    Output.omegaNavError(t/step+1,:) = [t, 180/pi*vectorNorm(angrateTrue - angrateEst), 180/pi*vectorNorm(angrateTrue - angrateMeas)];
    Output.power(t/step+1,:) = [t, powerCoils, energyCoils/3600]; % conversion of energy from [J] to [Wh] 
                
end

end

