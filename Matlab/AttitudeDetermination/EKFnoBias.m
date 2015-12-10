function [X, P] = EKFnoBias(X_k, P_k, Z, eclipse, b_eci, r_e2s_eci)
% Extended Kalman Filter with no bias estimation
%
% State Vector:
% X = [q1 q2 q3 q4 angrateX angrateY angrateZ]'
%
% Error State Vector:
% deltaX = [dq1 dq2 dq3 dAngrateX dAngrateY dAngrateZ]'
%
% Measurement Vector:
% Z = [magField' SunVector' angrate']
% magField measured is not unit!

global KalmanCovariance Inertia_ref step;

%% PREDICTION
q_k = X_k(1:4);
angrate_k = X_k(5:7);

% Propagate state vector using RK4
timerange = [0, step/2, step];
stateVector0 = [q_k', angrate_k'];
[~, stateVectorPredict] = ode45(@PropagateState, timerange, stateVector0);
qPredict = qnorm(stateVectorPredict(end,1:4)'); % normalization of predicted quaternion

% Discrete Jacobian for state transition
jacobianF = [-skew(angrate_k), 0.5*eye(3);
    zeros(3), Inertia_ref\(skew(Inertia_ref*angrate_k) - skew(angrate_k)*Inertia_ref)];
stateTrans = eye(6) + step * jacobianF;

% Prediction of covariance matrix P
P_predict = stateTrans * P_k * stateTrans' + KalmanCovariance.Q;

%% UPDATE
A_i2sPredict = q2m(qPredict);

magFieldPredict = A_i2sPredict * b_eci / vectorNorm(b_eci);
magFieldMeas = Z(1:3) / vectorNorm(Z(1:3));

sunVectorPredict = A_i2sPredict * r_e2s_eci;
if eclipse == 0
    sunVectorMeas = Z(4:6);    
else
    sunVectorMeas = sunVectorPredict;
end

angratePredict = stateVectorPredict(end, 5:7)';
angrateMeas = Z(7:9);

% Jacobian Measurement
H = [2*skew(magFieldPredict), zeros(3);
     2*skew(sunVectorPredict), zeros(3);
     zeros(3), eye(3)];

% Kalman Gain
K = P_predict * H' / (H*P_predict*H' + KalmanCovariance.R);

% Residual
delta_x = K * [(magFieldMeas - magFieldPredict)', (sunVectorMeas - sunVectorPredict)', (angrateMeas - angratePredict)']';

% Extract small rotation vector
smallRotVector = 2*delta_x(1:3);

% Expand quaternion & angrates
q = qPredict + 0.5 * [qPredict(4)*eye(3)+skew(qPredict(1:3)); -qPredict(1:3)'] * smallRotVector;
qNew = qnorm(q);

angrateNew = angratePredict + delta_x(4:6);

% Calculate outputs
X = [qNew; angrateNew];
P = (eye(6) - K*H) * P_predict;

end


function deriv = PropagateState(~, z)
% Right hand side function for RK4 procedure
% State Vector z = [q, angrate] where:
%  - q is attitude quaternion [q1, q2, q3, q4] where q4 is a scalar part
%    q represents attitude of SBRF wrt ECI
%  - angrate = [wx, wy, wz]
%    angrate is angular rate vector of SBRF wrt ECI expressed in SBRF

global Inertia_ref ctrlTorque_prev;

q = qnorm(z(1:4));
angrate = z(5:7);

deriv_q = 0.5 * [-skew(angrate), angrate; -angrate', 0] * q;
deriv_angrate = Inertia_ref \ (ctrlTorque_prev - skew(angrate)*(Inertia_ref*angrate)); 

deriv = [deriv_q; deriv_angrate];

end