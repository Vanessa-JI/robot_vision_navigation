% This script runs Q2(d)

clc;
clear;


% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Disable the GPS and enable the laser for pure SLAM.
configuration.enableGPS = false;
configuration.enableLaser = true;

% For this part of the coursework, this should be set to zero.
configuration.perturbWithNoise = false;

%% Q2d before loop closure
% Set this value to truncate the run at a specified timestep rather than
% run through the whole simulation to its end.
configuration.maximumStepNumber = 1205 ;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_d');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Add labels for simulation output
title('Simulation Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/q2_d_sim_out_before_closure.png')

% Plot optimisation times.
minislam.graphics.FigureManager.getFigure('Optimization times before closure');
clf
plot(results{1}.optimizationTimes, '*')
hold on
title('Optimization times before closure')
xlabel('Timestep No.')
ylabel('Optimisation Time (sec)')
saveas(gcf,'Figures/q2_d_optimisation_times_before_closure.png')

% Plot the error curves.
minislam.graphics.FigureManager.getFigure('Errors before closure');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')


% Plot covariance.
minislam.graphics.FigureManager.getFigure('Vehicle Covariances before closure');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on
legend('covariance in x', 'covariance in y', 'covariance in theta')
xlabel('Timestep No.')
title('Vehicle Covariances before closure')
ylabel('covariance')
saveas(gcf,'Figures/q2_d_covariances_before_closure.png')
fprintf("Determinant before closure")


% Plot errors.
minislam.graphics.FigureManager.getFigure('Errors before closure');
clf
errors = results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory';
% wrap theta in [-pi, pi]
errors(:,3) = g2o.stuff.normalize_thetas(errors(:,3));
plot(errors)
hold on
legend('error in x', 'error in y', 'error in theta')
title('Errors before closure')
xlabel('Timestep No.')
ylabel('error')
saveas(gcf,'Figures/q2_d_errors_before_closure.png')

%%Q2d


%% Q2d after loop closure

% Set this value to truncate the run at a specified timestep rather than
% run through the whole simulation to its end.
configuration.maximumStepNumber = 1208; % 1205 for before loop closure

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_d');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Add labels for simulation output
title('Simulation Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/q2_d_sim_out_aft_closure.png')

% Plot optimisation times.
minislam.graphics.FigureManager.getFigure('Optimization times after closure');
clf
plot(results{1}.optimizationTimes, '*')
hold on
title('Optimization times after closure')
xlabel('Timestep No.')
ylabel('Optimisation Time (sec)')
saveas(gcf,'Figures/q2_d_optimisation_times_aft_closure.png')
fprintf(results{1}.optimizationTimes)


% Plot the error curves.
minislam.graphics.FigureManager.getFigure('Errors after closure');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')


% Plot covariance.
minislam.graphics.FigureManager.getFigure('Vehicle Covariances after closure');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on
legend('covariance in x', 'covariance in y', 'covariance in theta')
xlabel('Timestep No.')
title('Vehicle Covariances after closure')
ylabel('covariance')
saveas(gcf,'Figures/q2_d_covariances_aft_closure.png')

fprintf("Determinant after closure")
disp(results{1}.vehicleCovarianceHistory)

% Plot errors.
minislam.graphics.FigureManager.getFigure('Errors after closure');
clf
errors = results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory';
% wrap theta in [-pi, pi]
errors(:,3) = g2o.stuff.normalize_thetas(errors(:,3));
plot(errors)
hold on
legend('error in x', 'error in y', 'error in theta')
title('Errors after closure')
xlabel('Timestep No.')
ylabel('error')
saveas(gcf,'Figures/q2_d_errors_aft_closure.png')
