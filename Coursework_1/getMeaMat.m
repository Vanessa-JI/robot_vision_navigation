function H_k = getMeaMat (nSat, X, U_aj)
% Compute measurement matrix H_k
%
% Inputs:
%   nSat        total number of satellites
%   X           8x1 state vector
%   U_aj        satellite-antenna line-of-sight unit vector
%
% Outputs
%   H_k     measurement matrix

H_k = zeros (2 * nSat, length (X));
H_k (1:nSat, 1:3) = -U_aj';
H_k (nSat+1 : end, 4:6) = -U_aj';
H_k (1:nSat, end-1) = 1;
H_k (nSat+1:end, end) = 1;
end