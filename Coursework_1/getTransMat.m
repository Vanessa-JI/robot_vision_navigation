function phi = getTransMat (tau_s)
% Compute the transition matrix
%
% Inputs:
%   tau_s       time interval in secs (int)
%
% Outputs
%   phi         transition matrix

phi = eye(8);
phi(1:3, 4:6) = eye(3) * tau_s;
phi (end-1, end) = tau_s;
end