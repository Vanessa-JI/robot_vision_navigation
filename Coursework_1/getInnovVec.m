function dZ = getInnovVec (RawData,r_aj, X, Omega_ie, U_aj, C_e, v_ej, r_ej, nSat, t)
% Compute measurement innovation vector
%
% Inputs:
%   RawData     struct containing raw data
%   r_aj        predicted satellite-antenna range vector
%   X           8x1 state vector
%   Omega_ie    skew-symmetric matrix
%   U_aj        satellite-antenna line-of-sight unit vector
%   C_e         Sagnac effect compensation matrix
%   r_ej        satellite-earth position vector
%   v_ej        satellite-earth velocity vector
%   nSat        total number of satellites
%   t           current runtime
%
% Outputs
%   dZ          measurement innovation vector

% Get predicted satellite-antenna range rate
rdot_aj = predictRangeRate (X, nSat, Omega_ie, U_aj, C_e, v_ej, r_ej);
% Get the current number of epoch
iEpoch = find(RawData.timeStamp==t);
% Formulate measurement innovation vector, dZ
dZ = [ (RawData.pseudoRanges(iEpoch,:) - r_aj - X(end - 1))';
    (RawData.pseudoRangeRates(iEpoch,:) - rdot_aj - X(end))'];
end