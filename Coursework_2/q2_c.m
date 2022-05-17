% This script runs Q2(c)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Disable the GPS and enable the laser for pure SLAM
configuration.enableGPS = false;
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_c');

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
saveas(gcf,'Figures/q2_c_sim_out.png')

% Add labels for simulation output
title('Simulation Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/q2_c_sim_out.png')

% Add labels for simulation output
title('Simulation Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/q2_c_sim_out.png')

% Add labels for simulation output
title('Simulation Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/q2_c_sim_out.png')

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
title('Optimization times')
xlabel('Timestep No.')
ylabel('Optimisation Time (sec)')
hold on
saveas(gcf,'Figures/q2_c_optimisation_times.png')

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
saveas(gcf,'Figures/q2_c_covariances.png')

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
saveas(gcf,'Figures/q2_c_errors.png')

% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on
title('Chi 2 values')
xlabel('Timestep No.')
ylabel('Chi2 Values')
saveas(gcf,'Figures/q2_c_chi2_values.png')

% This is how to extract the graph from the optimizer
graph = drivebotSLAMSystem.optimizer();

% This is how to extract cell arrays of the vertices and edges from the
% graph
allVertices = graph.vertices();
allEdges = graph.edges();


% Plot the chi2 value down here. The chi2 values are logged internally and
% can be plotted directly down here.
%warning('q2_c:unimplemented', 'Compute the diagnostics required for Q2c.');

%%%%%%%%% Q2c starts %%%%%%%%%%%%
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on
title('Chi 2 values')
xlabel('Time')
ylabel('Chi2 Values')
saveas(gcf,'Figures/q2_b_chi2_values.png');

% total number of vertices
numVertices = length(allVertices);

% total number of edges
numEdges = length(allEdges);

% number of vehicle poses stored
numVehPoses = length(results{1}.vehicleStateHistory');
fprintf('Total number of vehicle poses stored: %.0d. \n', numVehPoses)

% number of landmarks initalized
numLmVertices = numVertices - numVehPoses;
fprintf('Total number of landmarks initalized: %.0d. \n', numLmVertices)

% average number of observations made by a robot at each timestep
numObsVerEdge = numEdges - numVehPoses;
numObsPerDT = numObsVerEdge / numVehPoses;
fprintf(['Average number of observations made by a robot at' ...
    ' each timestep: %.2d. \n'], numObsPerDT)

% the average number of observations received by each landmark
numObsPerLM = numObsVerEdge / numLmVertices;
fprintf(['Average number of observations received by' ...
    ' each landmark: %.2d. \n'], numObsPerLM)

%%%%%%%%% Q2c ends %%%%%%%%%%%%

