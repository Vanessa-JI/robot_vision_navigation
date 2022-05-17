function GNSSResults = GNSSSolution(Pseudo_ranges_data, Pseudo_range_rates_data)
% Compute user geodetic position and velocity with GNSS measurement
%
% Inputs:
%   Pseudo_ranges_data      measured psuedo range data matrix
%   Pseudo_range_Rates_data     measured psuedo range rate data matrix
%
% Outputs:
%   GNSSResults     GNSS results struct with fields:
%       timeStamp       Time in seconds – this must match the data files.
%       latitude        Geodetic latitude in degrees
%       longitude       Geodetic longitude in degrees
%       velNorth        North velocity in metres per second
%       velEast         East velocity in metres per second
%       heading         Heading in degrees

% Everything to 15 decimal places using format long
format long

%% Import data

%RawData.tableRanges = (readmatrix("data/Pseudo_ranges.csv"));
%RawData.tableRates = (readmatrix("data/Pseudo_range_rates.csv"));
RawData.tableRanges = Pseudo_ranges_data;
RawData.tableRates = Pseudo_range_rates_data;
RawData.timeStamp = RawData.tableRanges(2:end, 1);
RawData.satNos = RawData.tableRanges(1, 2:end);
RawData.pseudoRanges = RawData.tableRanges(2:end, 2:end);
RawData.pseudoRangeRates = RawData.tableRates(2:end, 2:end);


%% Initialisation

Define_Constants;

% Get satellite number
nSat = length(RawData.satNos);

% Compute inital error covariance matrix
P_k = [sdOfs^2 * ones(3,1);
    sdDft^2 * ones(3,1);
    sdOfs^2;
    sdDft^2];
P_k = diag(P_k);

% initial state
X = zeros(8,1); %[x y z v_x v_y v_z rClockOffset rClockDrift]


%% First epoch
xNorm = 0;
diff = 1;
threshold = 1e-5;

% Compute first epoch of least squares solution
t=0;
[X, H_k, dZ] = getGNSSLeastSquare(RawData, X, nSat, omega_ie, Omega_ie, c, diff, xNorm, threshold, t);

% Detect and remove outliers
ifOutlier = true;

while ifOutlier == true
    [RawData, nSat, ifOutlier] = detectOutliers (RawData, nSat, ifOutlier, H_k, dZ);
    % Compute least squares solution with outliers removed
    [X, H_k, dZ] = getGNSSLeastSquare(RawData, X, nSat, omega_ie, Omega_ie, c, diff, xNorm, threshold, t);
end

% Convert ECEF to NED
[L_b,lambda_b,h,v_eb_n] = pv_ECEF_to_NED(X(1:3), X(4:6));

% Record results
GNSSResults.timeStamp = 0 * ones(1,1);
GNSSResults.latitude = L_b * rad_to_deg * ones(1,1);
GNSSResults.longitude = lambda_b * rad_to_deg * ones(1,1);
GNSSResults.velNorth = v_eb_n(1) * ones(1,1);
GNSSResults.velEast = v_eb_n(2) * ones(1,1);
GNSSResults.heading = atan2 (v_eb_n(2), v_eb_n(1)) * rad_to_deg * ones(1,1);
GNSSResults.altitude = h;

%% Multi-epoch with Kalman Filter
for t = RawData.timeStamp(2):tau :RawData.timeStamp(end)
    % Step 1: Compute the transition matrix
    phi = getTransMat (tau);

    % Step 2: Compute the system noise covariance matrix
    Q = getSysNoiseCovMat (S_a, tau, S_cf, S_cphi);

    % Step 3: Use the transition matrix to propagate the state estimates
    X = phi * X;

    % Step 4: Propagate the error covariance matrix
    P_k = phi * P_k * phi' + Q;

    % Predict the ranges from the approximate user position to each satellite
    [r_aj, C_e, r_ej, v_ej] = predictRanges(RawData, X, nSat, omega_ie, c, t);

    % Compute the line-of-sight unit vector from the approximate user position to each satellite
    U_aj = getLineOfSightVec (nSat, C_e, r_ej, r_aj, X);

    % Step 5: Compute the measurement matrix
    H_k = getMeaMat (nSat, X, U_aj);

    % Step 6: Compute the measurement noise covariance matrix
    R_k = [eye(nSat) * sdPseuRanMea^2, zeros(nSat);
        zeros(nSat), eye(nSat) * sdPseuRanRateMea^2];

    % Step 7: Compute the Kalman gain matrix
    K_k = P_k * H_k' / (H_k * P_k * H_k' + R_k);

    % Step 8: Formulate the measurement innovation vector, dZ
    dZ = getInnovVec (RawData,r_aj, X, Omega_ie, U_aj, C_e, v_ej, r_ej, nSat, t);

    % Step 9: Update the state estimates
    X = X + K_k * dZ;

    % Step 10: Update the error covariance matrix
    P_k = (eye(8) - K_k * H_k) * P_k;

    % Convert ECEF to NED
    [L_b,lambda_b,h,v_eb_n] = pv_ECEF_to_NED(X(1:3), X(4:6));

    % Record results
    GNSSResults.timeStamp (end+1) = t;
    GNSSResults.latitude (end+1) = L_b * rad_to_deg;
    GNSSResults.longitude (end+1) = lambda_b * rad_to_deg;
    GNSSResults.velNorth (end+1) = v_eb_n(1);
    GNSSResults.velEast (end+1) = v_eb_n(2);
    GNSSResults.heading (end+1) = atan2 (v_eb_n(2), v_eb_n(1)) * rad_to_deg;
    GNSSResults.altitude (end+1) = h;
end

% Convert each field to column vector
fns = fieldnames(GNSSResults);
for iField = 1:numel (fns)
    GNSSResults.(fns{iField}) = GNSSResults.(fns{iField})';
end

writetable(struct2table(GNSSResults), 'Results/GNSSResults.xlsx')

end



