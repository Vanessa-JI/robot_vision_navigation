%Define_Constants
%SCRIPT This defines a number of constants for your use
% Created 16/11/16 by Paul Groves

% Copyright 2016, Paul Groves
% License: BSD; see license.txt for details
% Modified for coursework by Joseph Rowell, Vanessa Igodifo, Jason Zhang

% Constants
deg_to_rad = 0.01745329252; % Degrees to radians conversion factor
rad_to_deg = 1/deg_to_rad; % Radians to degrees conversion factor
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
Omega_ie = Skew_symmetric([0,0,omega_ie]);
R_0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity

% GNSS Error
sdSignal = 1; % Signal in space error standard deviation: 1m.
sdIon = 2;    % Residual ionosphere error standard deviation at zenith: 2m.
sdTro = 0.2;  % Residual troposphere error standard deviation at zenith: 0.2m.
sdCodeTrack = 2; % Code tracking and multipath error standard deviation: 2m.
sdRateTrack = 0.02; % Range rate tracking and multipath error standard deviation: 0.02 m/s.

% Measurement error
sdPseuRanMea= 10; % standard deviation of pseudo range measurment
sdPseuRanRateMea = 0.05; % standard deviation of pseudo range measurment

% Combine measurement error with other GNSS errors
sdPseuRanMea = sqrt (sdPseuRanMea^2 + sdPseuRanMea^2 + sdIon^2 + sdCodeTrack^2);
sdPseuRanRateMea = sqrt (sdPseuRanRateMea^2 + sdRateTrack^2);

% Clock error
sdOfs = 100000; % standard deviation of receiver clock offset
sdDft = 200; % standard deviation of receiver clock drift
S_cphi = 0.01 ; % clock phase PSD (power spectral density)
S_cf = 0.04 ; % clock frequency PSD
S_a = 0.01; % acceleration PSD

% Gyro error
gyro_bias_std = 1; % A bias standard deviation of 1 degree per second
gyro_scale_factor_err_std = 0.01; % A scale factor error standard deviation of 1%.
gyro_cross_coup_err_std = 0.001; % A cross-coupling error standard deviation of 0.1%.
gyro_noise_std = 10e-4; %Noise standard deviation of 10e-4 rad/s
gyro_quant_level = 2e-4; % Quantisation level  2e-4 rad/s

tau = 0.5;  % time interval

PSD = S_a;

% Ends
