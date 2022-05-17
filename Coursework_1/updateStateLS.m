function X = updateStateLS (X, H_k, dZ)
% Update state vector using unweighted least-squares
%
% Inputs:
%   X       8x1 state vector [x y z v_x v_y v_z rClockOffset rClockDrift]'
%   H_k     measurement matrix
%   dZ      measurement innovation vector
%
% Outputs:
%   X       Update state vector


X = X + (H_k' * H_k) \ H_k' * dZ;
end
