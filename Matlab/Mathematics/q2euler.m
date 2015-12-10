function euler = q2euler(q)

% Calculating Euler angles [deg] from quaternion [q1 q2 q3 q0]'
pitch = asin(-2 * (q(1) * q(3) - q(4) * q(2))) * 180 / pi;
yaw = atan2(2 * (q(1) * q(2) + q(4) * q(3)), q(1) * q(1) - q(2) * q(2) - q(3) * q(3) + q(4) * q(4)) * 180 / pi;
roll = atan2(2 * (q(2) * q(3) + q(1) * q(4)), -q(1) * q(1) - q(2) * q(2) + q(3) * q(3) + q(4) * q(4)) * 180 / pi;

euler = [yaw, pitch, roll]'; % Euler angles in [deg]

end