% This script runs Q3(a)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% SLAM is enabled, GPS disabled
configuration.enableGPS = false;
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Magic tuning for the no-prediction case
configuration.laserDetectionRange = 30;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q3_a');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Optimize every 500 timesteps to get a picture of the situation evolves
drivebotSLAMSystem.setRecommendOptimizationPeriod(500);

% Set whether the SLAM system should remove prediction edges. If the first
% value is true, the SLAM system should remove the edges. If the second is
% true, the first prediction edge will be retained.
drivebotSLAMSystem.setRemovePredictionEdges(true, true);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
hold on
title('Optimization times')
xlabel('Timestep No.')
ylabel('Optimisation Time (sec)')
saveas(gcf,'Figures/q3_a_optimisation_times.png')

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on
legend('covariance in x', 'covariance in y', 'covariance in theta')
xlabel('Timestep No.')
title('Vehicle Covariances')
ylabel('covariance')
saveas(gcf,'Figures/q3_a_covariances.png')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
% wrap theta in [-pi, pi]
errors = results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory';
errors(:,3) = g2o.stuff.normalize_thetas(errors(:,3));
plot(errors)
% plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
hold on
legend('error in x', 'error in y', 'error in theta')
title('Errors')
xlabel('Timestep No.')
ylabel('error')
saveas(gcf,'Figures/q3_a_errors.png')


% Plot the chi2 value down here. The chi2 values are logged internally and
% can be plotted directly down here.
%warning('q3_a:unimplemented', 'Compute the diagnostics required for Q3a.');
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on
title('Chi 2 values')
xlabel('Timestep No.')
ylabel('Chi2 Values')
saveas(gcf,'Figures/q3_a_chi2_values.png')

