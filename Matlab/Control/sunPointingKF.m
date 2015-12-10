function dipoleComm = sunPointingKF(angrate_s, r_e2s, qEst, b_sEst)
% Calculating desired control dipole for Sun pointing mode [Am2]
% desired control torque is expressed in satellite frame
% control law is based on spin stabilization
% underactuation of the coils is included

global Inertia_ref ctrlgains;

%% Commanded angular rates (using sun vector measurement)
angrate_comm = ctrlgains.angrate;
angrate_comm_norm = vectorNorm(angrate_comm);
A_i2s = q2m(qEst);
angrate_comm_s = angrate_comm_norm * A_i2s * r_e2s; % in satellite frame (current orientation)


%% Control torque
% angular momentum error
K_error = Inertia_ref * (angrate_comm_s - angrate_s);

% precession error
P_error = Inertia_ref(1,1) * (angrate_comm_norm - angrate_s(1));

% control law
k = ctrlgains.k;
kp = ctrlgains.kp;
kn = ctrlgains.kn;

ctrltorq = k * K_error + kp * P_error * [1 0 0]' + kn * diag([0,1,1]) * angrate_s;

%% Command magnetic dipole
bnorm = vectorNorm(b_sEst);
dipoleComm = skew(b_sEst) * ctrltorq / bnorm^2;

end