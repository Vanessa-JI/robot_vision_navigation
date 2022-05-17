function Q = getSysNoiseCovMat (S_a, tau_s, S_cf, S_cphi)
% Compute the system noise covariance matrix
%
% Inputs:
%   S_a         acceleration PSD (float)
%   tau_s       time interval (float)
%   S_cf        clock frequency PSD (float)
%   S_cphi      clock phase PSD (float)
%
% Outputs
%   Q           systen noise covariance matrix

Q = zeros (8);
Q (1:3, 1:3) = 1/3 * eye(3) * S_a * tau_s^3;
Q (1:3, 4:6) = 1/2 * eye(3) * S_a * tau_s^2;
Q (4:6, 1:3) = 1/2 * eye(3) * S_a * tau_s^2;
Q (4:6, 4:6) = eye(3) * S_a * tau_s;
Q (7, 7) = S_cphi * tau_s + 1/3 * S_cf * tau_s^3;
Q (7, 8) = 1/2 * S_cf * tau_s^2;
Q (8, 7) = 1/2 * S_cf * tau_s^2;
Q (8, 8) = S_cf * tau_s;
end