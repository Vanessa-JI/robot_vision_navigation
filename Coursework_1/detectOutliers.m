function [RawData, nSat, ifOutlier] = detectOutliers(RawData, nSat, ifOutlier, H_k, dZ)
% Detect outliers and remove the largest one
%
% Inputs:
%   RawData     struct containing raw data
%   nSat        total number of satellites
%   ifOutlier   outlier varifying boolean
%   H_k     measurement matrix
%   dZ      measurement innovation vector
%
% Outputs
%   RawData     struct containing raw data
%   nSat        total number of satellites
%   ifOutlier   outlier varifying boolean


H_k = [H_k(1:nSat, 1:3), ones(nSat, 1)];
dZ = dZ(1:nSat);

% compute residuals vector
v = ((H_k / (H_k' * H_k) * H_k') - eye(nSat)) * dZ;

% compute residuals covariance matrix
sigma_p = 5;
C_v = (eye(nSat) - H_k / (H_k' * H_k) * H_k') * sigma_p^2;

% compute normalised residuals and compare to threshold
T = 6; % threshold
maxResidual = 0;
ind = 0;

% Check every residual and index the satellite corresponding to the largest
% outlier
for i = 1:nSat
    outlierCheck = sqrt(C_v(i, i)) * T;
    residual = abs(v(i));
    if residual > outlierCheck
        if residual > maxResidual
            maxResidual = residual;
            ind = find(abs(v) == maxResidual);
        end
    end
end

% Check if any outlier exists
if ind == 0
    ifOutlier = false;
end

if ifOutlier == true
    % remove whole column in dataset
    RawData.pseudoRanges = RawData.pseudoRanges(:, [setdiff(1:nSat, ind)]);
    RawData.pseudoRangeRates = RawData.pseudoRangeRates(:, [setdiff(1:nSat, ind)]);
    RawData.satNos = RawData.satNos(:, [setdiff(1:nSat, ind)]);

    % updating the number of satellites
    nSat = nSat - 1;
end

end