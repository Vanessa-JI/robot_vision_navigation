%Second iteration of cw1_script
%Script to execute COMP0130 ROBOT VISION AND NAVIGATION
%Coursework 1: Integrated Navigation for a Robotic Lawnmower
%Authors: Yuangshen (Jason) Zhang, Joseph Rowell & Vanessa Igodifo (Equal
%contributions)

close all;
clear;
clc;

%Import useful constants
Define_Constants;
%Set long format so numbers reported to 15 d.p
format long;

%% GNSS SOLUTION
% Import GNSS data
Pseudo_ranges_data = (readmatrix("data/Pseudo_ranges.csv"));
Pseudo_rates_data = (readmatrix("data/Pseudo_range_rates.csv"));
%Compute GNSS solution
GNSSResults = GNSSSolution(Pseudo_ranges_data, Pseudo_rates_data);

%% DEAD RECKONING SOLUTION
% Import dead reckoning data
dead_reckoning_data = csvread('data/Dead_reckoning.csv');
% Compute dead reckoning Solution
deadReckoningResults = deadReckoning(dead_reckoning_data, GNSSResults);

%% INTEGRATED SOLUTION
% % Compute integrated solution 
integratedResults = IntegratedSolution(dead_reckoning_data, ...
    deadReckoningResults, Pseudo_rates_data, Pseudo_rates_data, GNSSResults);
% 
% %% GNSS PLOTTING
plottingFunction('GNSS', GNSSResults.timeStamp, ...
    GNSSResults.longitude, GNSSResults.latitude, GNSSResults.velNorth, GNSSResults.velEast);

% %% Dead Reckoning PLOTTING
plottingFunction('Dead Reckoning', deadReckoningResults.times, ...
    deadReckoningResults.longitudes, deadReckoningResults.latitudes, deadReckoningResults.vel_N, deadReckoningResults.vel_E);
 

%% INTEGRATED SOLUTION PLOTTING
plottingFunction('Integrated Solution', integratedResults.times, ...
    integratedResults.longitudes, integratedResults.latitudes, integratedResults.vel_N, integratedResults.vel_E);