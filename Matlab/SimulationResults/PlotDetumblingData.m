function PlotDetumblingData(Output, T)
% Plot Data for Detumbling Mode

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
saveas(h(1), [files_path, '/simulationresults/detumbling/Position_ECI.png'])

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
saveas(h(2), [files_path, '/simulationresults/detumbling/Velocity_ECI.png'])

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
saveas(h(3), [files_path, '/simulationresults/detumbling/IGRF11_10th_ECI.png'])

h(4) = figure;
plot(Output.trueStateVector(:,1)/T, Output.trueStateVector(:,5), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.trueStateVector(:,1)/T, Output.trueStateVector(:,6), 'r-', 'LineWidth', 1)
hold on
plot(Output.trueStateVector(:,1)/T, Output.trueStateVector(:,7), 'g-', 'LineWidth', 1)
legend('X', 'Y', 'Z')
title('True Angular Rates of Satellite in SBRF for Control Input based on Raw MTM Data [deg/s]')
xlabel('No. of Orbits')
saveas(h(4), [files_path, '/simulationresults/detumbling/AngRate_SBRF.png'])

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
plot(Output.torques(:,1)/T, Output.torques(:,6), 'm-', 'LineWidth', 1)
hold on
grid on
plot(Output.torques(:,1)/T, Output.torques(:,5), 'k-', 'LineWidth', 1)
legend('Control Torque', 'Magnetic Disturbance Torque')
xlabel('No. of Orbits')
saveas(h(6), [files_path, '/simulationresults/detumbling/Torques.png'])

h(7) = figure;
plot(Output.magneticDipole(:,1)/T, Output.magneticDipole(:,2), 'b-', 'LineWidth', 1)
hold on
grid on
stairs(Output.magneticDipole(:,1)/T, Output.magneticDipole(:,3), 'r-', 'LineWidth', 1)
legend('Disturbance Magnetic Moment', 'Control Magnetic Moment')
title('Norm of Magnetic Moment [Am2]')
xlabel('No. of Orbits')
saveas(h(7), [files_path, '/simulationresults/detumbling/MagneticDipoles_Dist_Ctrl.png'])

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
saveas(h(8), [files_path, '/simulationresults/detumbling/MagneticDipole_Coils.png'])

h(9) = figure;
plot(Output.BdotRaw(:,1)/T, Output.BdotRaw(:,2), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.BdotRaw(:,1)/T, Output.BdotRaw(:,3), 'r-', 'LineWidth', 1)
hold on
plot(Output.BdotRaw(:,1)/T, Output.BdotRaw(:,4), 'g-', 'LineWidth', 1)
title('B-Dot Vector (High-Pass Filtered MTM Data) in SBRF [nT/s]')
legend('X', 'Y', 'Z')
xlabel('No. of Orbits')
saveas(h(9), [files_path, '/simulationresults/detumbling/BDot_Measurement.png'])

h(10) = figure;
plot(Output.angRateNorm(:,1)/T, Output.angRateNorm(:,2), 'b-', 'LineWidth', 1)
grid on
title('Angular Rate Vector Norm [deg/s]')
xlabel('No. of Orbits')
saveas(h(10), [files_path, '/simulationresults/detumbling/AngRate_Norm.png'])

h(11) = figure;
stairs(Output.power(:,1)/T, Output.power(:,2), 'b-', 'LineWidth', 1)
hold on
grid on
plot(Output.power(:,1)/T, Output.power(:,3), 'r-', 'LineWidth', 1)
axis tight
title('Power & Energy Consumption for Coils Actuation')
legend('Power [W]', 'Energy [Wh]')
xlabel('No. of Orbits')
saveas(h(11), [files_path, '/simulationresults/detumbling/Power_Energy.png'])