function PlotSunPointingData(Output, T)
% Plot Data for Sun Pointing Mode

global files_path;

h(1) = figure;
plot(Output.orbitPar(:,1)/T, Output.orbitPar(:,2), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.orbitPar(:,1)/T, Output.orbitPar(:,3), 'r-', 'LineWidth', 1)
hold on
plot(Output.orbitPar(:,1)/T, Output.orbitPar(:,4), 'g-', 'LineWidth', 1)
legend('X', 'Y', 'Z')
title('Satellite Position in ECI [km]')
xlabel('No. of Orbits')
saveas(h(1), [files_path, '/simulationresults/sunpointing/Position_ECI.png'])

h(2) = figure;
plot(Output.orbitPar(:,1)/T, Output.orbitPar(:,5), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.orbitPar(:,1)/T, Output.orbitPar(:,6), 'r-', 'LineWidth', 1)
hold on
plot(Output.orbitPar(:,1)/T, Output.orbitPar(:,7), 'g-', 'LineWidth', 1)
legend('X', 'Y', 'Z')
title('Satellite Velocity in ECI [km/s]')
xlabel('No. of Orbits')
saveas(h(2), [files_path, '/simulationresults/sunpointing/Velocity_ECI.png'])

h(3) = figure;
plot(Output.magneticField(:,1)/T, Output.magneticField(:,2), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.magneticField(:,1)/T, Output.magneticField(:,3), 'r-', 'LineWidth', 1)
hold on
plot(Output.magneticField(:,1)/T, Output.magneticField(:,4), 'g-', 'LineWidth', 1)
legend('X', 'Y', 'Z')
title('Magnetic Field IGRF11 10th Order in ECI [nT]')
xlabel('No. of Orbits')
saveas(h(3), [files_path, '/simulationresults/sunpointing/IGRF11_10th_ECI.png'])

h(5) = figure;
plot(Output.trueStateVector(:,1)/T, Output.trueStateVector(:,5), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.trueStateVector(:,1)/T, Output.trueStateVector(:,6), 'r-', 'LineWidth', 1)
hold on
plot(Output.trueStateVector(:,1)/T, Output.trueStateVector(:,7), 'g-', 'LineWidth', 1)
legend('X', 'Y', 'Z')
title('True Angular Rates of Satellite in SBRF for Control Input based on EKF [deg/s]')
xlabel('No. of Orbits')
saveas(h(5), [files_path, '/simulationresults/sunpointing/AngRate_SBRF.png'])

h(6) = figure;
subplot(2,1,1)
plot(Output.torques(:,1)/T, Output.torques(:,2), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.torques(:,1)/T, Output.torques(:,3), 'r-', 'LineWidth', 1)
hold on
plot(Output.torques(:,1)/T, Output.torques(:,4), 'g-', 'LineWidth', 1)
legend('Aerodynamic Torque', 'Gravity Gradient Torque', 'Solar Radiation Torque')
title('Norms of Torques [Nm]')
xlabel('No. of Orbits')
subplot(2,1,2)
plot(Output.torques(:,1)/T, Output.torques(:,5), 'k-', 'LineWidth', 1)
hold on
grid on
plot(Output.torques(:,1)/T, Output.torques(:,6), 'm-', 'LineWidth', 1)
legend('Magnetic Disturbance Torque', 'Control Torque')
xlabel('No. of Orbits')
saveas(h(6), [files_path, '/simulationresults/sunpointing/Torques.png'])

h(7) = figure;
plot(Output.magneticDipole(:,1)/T, Output.magneticDipole(:,2), 'b-', 'LineWidth', 1)
hold on
grid on
stairs(Output.magneticDipole(:,1)/T, Output.magneticDipole(:,3), 'r-', 'LineWidth', 1)
legend('Disturbance Magnetic Moment', 'Control Magnetic Moment')
title('Norm of Magnetic Moment [Am2]')
xlabel('No. of Orbits')
saveas(h(7), [files_path, '/simulationresults/sunpointing/MagneticDipoles_Dist_Ctrl.png'])

h(8) = figure;
stairs(Output.dipoleApllied(:,1)/T, Output.dipoleApllied(:,2), 'b-', 'LineWidth', 1)
hold on
grid on
stairs(Output.dipoleApllied(:,1)/T, Output.dipoleApllied(:,3), 'r-', 'LineWidth', 1)
hold on
stairs(Output.dipoleApllied(:,1)/T, Output.dipoleApllied(:,4), 'g-', 'LineWidth', 1)
title('Magnetic Dipole Applied by Coils in SBRF [Am2]')
legend('X', 'Y', 'Z')
xlabel('No. of Orbits')
saveas(h(8), [files_path, '/simulationresults/sunpointing/MagneticDipole_Coils.png'])

h(9) = figure;
plot(Output.omegaCtrlError(:,1)/T, Output.omegaCtrlError(:,2), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.omegaCtrlError(:,1)/T, Output.omegaCtrlError(:,3), 'r-', 'LineWidth', 1)
hold on
plot(Output.omegaCtrlError(:,1)/T, Output.omegaCtrlError(:,4), 'g-', 'LineWidth', 1)
title('Satellite Angular Rate Error in SBRF (Command - True) [deg/s]')
legend('X', 'Y', 'Z')
xlabel('No. of Orbits')
saveas(h(9), [files_path, '/simulationresults/sunpointing/AngRate_Error_Comm_True.png'])

h(10) = figure;
plot(Output.angleError(:,1)/T, Output.angleError(:,2), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.angleError(:,1)/T, Output.angleError(:,3), 'r-', 'LineWidth', 1)
title('Angle between Satellite X Axis, Angular Rate Vector and Sun Direction [deg]')
legend('Sun Pointing Error', 'Angular Rate Error')
xlabel('No. of Orbits')
saveas(h(10), [files_path, '/simulationresults/sunpointing/SunPointing_Error.png'])

h(11) = figure;
plot(Output.attNavError(:,1)/T, Output.attNavError(:,3), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.attNavError(:,1)/T, Output.attNavError(:,2), 'r-', 'LineWidth', 1)
title('Attitude Error - Small Rotation Vector Norm [deg]')
legend('Attitude Error between True & TRIAD (gyro in eclipse)', 'Attitude Error between True & EKF')
xlabel('No. of Orbits')
ylim([0, 40])
saveas(h(11), [files_path, '/simulationresults/sunpointing/Attitude_Error.png'])

h(12) = figure;
plot(Output.omegaNavError(:,1)/T, Output.omegaNavError(:,3), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.omegaNavError(:,1)/T, Output.omegaNavError(:,2), 'r-', 'LineWidth', 1)
title('Angular Rate Error - Vector Norm [deg/s]')
legend('Error between True & Raw', 'Error between True & EKF')
xlabel('No. of Orbits')
saveas(h(12), [files_path, '/simulationresults/sunpointing/AngRate_Error.png'])

h(13) = figure;
stairs(Output.power(:,1)/T, Output.power(:,2), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.power(:,1)/T, Output.power(:,3), 'r-', 'LineWidth', 1)
axis tight
title('Power & Energy Consumption for Coils Actuation')
legend('Power [W]', 'Energy [Wh]')
xlabel('No. of Orbits')
saveas(h(13), [files_path, '/simulationresults/sunpointing/Power_Energy.png'])