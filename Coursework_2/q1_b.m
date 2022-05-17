% This script runs Q1(b)
clear;
clc;

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Since we are doing just prediction, disable the GPS and SLAM sensors.
configuration.enableGPS = false;
configuration.enableLaser = false;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator. The second argument points to the directory which
% contains the data used by the simulator.
simulator = drivebot.DriveBotSimulator(configuration, 'q1_bc');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% The code below generates minimal output plots. Please remember to label
% your axes and title your figures.

% Add labels for simulation output
title('Simulation Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/q1_b_sim_out.png')

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
title('Optimization times')
xlabel('Timestep No.')
ylabel('Optimisation Time (sec)')
hold on
saveas(gcf,'Figures/q1_b_optimisation_times.png')
% % Plot the error curves
% minislam.graphics.FigureManager.getFigure('Errors');
% clf
% plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance 
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on
legend('covariance in x', 'covariance in y', 'covariance in theta')
title('Vehicle Covariances')
xlabel('Timestep No.')
ylabel('covariance')
saveas(gcf,'Figures/q1_b_covariance.png')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
errors = results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory';
% wrap theta in [-pi, pi]
errors(:,3) = g2o.stuff.normalize_thetas(errors(:,3));
plot(errors)
hold on
legend('error in x', 'error in y', 'error in theta')
title('Errors')
xlabel('Timestep No.')
ylabel('error')
saveas(gcf,'Figures/q1_b_errors.png')

