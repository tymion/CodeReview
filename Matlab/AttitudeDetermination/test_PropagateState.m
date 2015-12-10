clc;
clear;
z = [0.2315, 0.4629, 0.6944, 0.5, 0.11, 0.22, -0.15]';
global ctrlTorque_prev Inertia_ref;

ctrlTorque_prev = 1e-8*[1, 2, 3]';
Inertia_ref = 1e-4*[1,2,3;2,5,1;3,1,9];

PropagateState(z)