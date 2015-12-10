function [X, P, res, resCov] = EKFsunVector(X_k, P_k, Z, eclipse, b_eci, r_e2s_eci, time_step)
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
% CAUTION: magField measured is not unit!
%
% res [9x1] residual : measurement - expected measurement
% resCov [9x9] residual covariance

global KalmanCovariance Inertia_ref;

%% PREDICTION
q_k = X_k(1:4);
angrate_k = X_k(5:7);

% Propagate state vector using RK4
k1 = PropagateState(X_k);
k2 = PropagateState(X_k + 0.5*time_step*k1);
k3 = PropagateState(X_k + 0.5*time_step*k2);
k4 = PropagateState(X_k + time_step*k3);
stateVectorPredict = X_k + time_step/6 * (k1 + 2*k2 + 2*k3 + k4);

% timerange = [0, step/2, step];
% stateVector0 = [q_k', angrate_k'];
% [~, stateVectorPredict] = ode45(@PropagateState, timerange, stateVector0);
qPredict = qnorm(stateVectorPredict(1:4)); % normalization of predicted quaternion

% Discrete Jacobian for state transition
jacobianF = [-skew(angrate_k), eye(3);
    zeros(3), Inertia_ref\(skew(Inertia_ref*angrate_k) - skew(angrate_k)*Inertia_ref)];
stateTrans = eye(6) + time_step * jacobianF;

% Prediction of covariance matrix P
P_predict = stateTrans * P_k * stateTrans' + time_step*KalmanCovariance.Q;

%% UPDATE
A_i2sPredict = q2m(qPredict);

magFieldPredict = A_i2sPredict * b_eci;
magFieldMeas = Z(1:3);

sunVectorPredict = A_i2sPredict * r_e2s_eci;
if eclipse == 0
    sunVectorMeas = Z(4:6);    
else
    sunVectorMeas = sunVectorPredict;
end

angratePredict = stateVectorPredict(5:7);
angrateMeas = Z(7:9);

% Jacobian Measurement
H = [skew(magFieldPredict), zeros(3);
     skew(sunVectorPredict), zeros(3);
     zeros(3), eye(3)];
 
% Kalman Gain
resCov = H*P_predict*H' + KalmanCovariance.R;
P_predict * H'
K = P_predict * H' / resCov

if eclipse == 1
    K(:,4:6) = zeros(6,3);
end

% Residual
res = [(magFieldMeas - magFieldPredict)', (sunVectorMeas - sunVectorPredict)', (angrateMeas - angratePredict)']';
delta_x = K * res;

% Extract small rotation vector
smallRotVector = delta_x(1:3);

% Expand quaternion & angrates
q = qPredict + 0.5 * [qPredict(4)*eye(3)+skew(qPredict(1:3)); -qPredict(1:3)'] * smallRotVector;
qNew = qnorm(q);

angrateNew = angratePredict + delta_x(4:6);

% Calculate outputs
X = [qNew; angrateNew];
jf_m = eye(6) - K*H;
P = jf_m*P_predict*jf_m' + K*KalmanCovariance.R*K'; % Joseph form

end
