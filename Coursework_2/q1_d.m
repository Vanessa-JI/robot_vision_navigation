% This script runs Q1(d)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Since we are doing prediction and GPS, disable the SLAM sensor
configuration.enableGPS = true;
configuration.enableLaser = false;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q1_d');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% Q1d:
drivebotSLAMSystem.setRecommendOptimizationPeriod(1);
%warning('q1_d:unimplemented', 'Change the update rate to 1 for Q1d.');

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
saveas(gcf,'Figures/q1_d_sim_out.png')

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
title('Optimization times')
xlabel('Timestep No.')
ylabel('Optimisation Time (sec)')
hold on
saveas(gcf,'Figures/q1_d_optimisation_times.png')

% Plot the error curves
% minislam.graphics.FigureManager.getFigure('Errors');
% clf
% plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on
legend('covariance in x', 'covariance in y', 'covariance in theta')
xlabel('Timestep No.')
title('Vehicle Covariances')
ylabel('covariance')
saveas(gcf,'Figures/q1_d_covariances.png')

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
saveas(gcf,'Figures/q1_d_errors.png')

% Plot the chi2 value down here. The chi2 values are logged internally and
% can be plotted directly down here.
%error('q1_d:unimplemented', 'Plot the chi2 values for Q1d.'); DONE 
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on
title('Chi 2 values')
xlabel('Time')
ylabel('Chi2 Values')
saveas(gcf,'Figures/q1_d_chi2_values.png')

