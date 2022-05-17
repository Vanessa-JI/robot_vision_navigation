function [X, H_k, dZ, U_aj, r_aj, C_e, r_ej, v_ej] = ...
    getGNSSLeastSquare(RawData, X, nSat, omega_ie, Omega_ie, c, diff, xNorm, threshold, t)
% Compute GNSS least squares solution for one epoch
%
% Inputs:
%   RawData     struct containing raw data
%   X           8x1 state vector
%   nSat        total number of satellites
%   omega_ie    Earth rotation rate in rad/s
%   Omega_ie    skew-symmetric matrix
%   c           Speed of light in m/s
%   diff        difference between cureent position to the previous
%                  iteration (long float)
%   xNorm       L2 norm of positions
%   threshold   convergence threshold (float)
%   t           current runtime
%
% Outputs:
%   X       Update state vector
%   H_k     measurement matrix
%   dZ      measurement innovation vector
%   U_aj    satellite-antenna line-of-sight unit vector
%   r_aj        predicted satellite-antenna range vector
%   C_e         Sagnac effect compensation matrix
%   r_ej        satellite-earth position vector
%   v_ej        satellite-earth velocity vector

while diff > threshold
    % Compute new state
    [r_aj, C_e, r_ej, v_ej] = predictRanges(RawData, X, nSat, omega_ie, c, t);
    U_aj = getLineOfSightVec (nSat, C_e, r_ej, r_aj, X);
    dZ = getInnovVec (RawData,r_aj, X, Omega_ie, U_aj, C_e, v_ej, r_ej, nSat, t);
    H_k = getMeaMat (nSat, X, U_aj);
    X = updateStateLS (X, H_k, dZ);
    % Compare the current state to the previous iteration
    xNormPrev = xNorm;
    xNorm = norm(X(1:3));
    diff = abs(xNorm - xNormPrev);
end
end