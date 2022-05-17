function rdot_aj = predictRangeRate (X, nSat, Omega_ie, U_aj, C_e, v_ej, r_ej)
% Predict range rate from approximate user position to each satellite
%
% Inputs:
%   X           8x1 state vector
%   nSat        total number of satellites
%   Omega_ie    skew-symmetric matrix
%   C_e         Sagnac effect compensation matrix
%   r_ej        satellite-earth position vector
%   v_ej        satellite-earth velocity vector
%   U_aj        satellite-antenna line-of-sight unit vector
%
% Outputs
%   rdot_aj     Predicted satellite-user range rate vector

rdot_aj = zeros(1, nSat);
for jSat = 1: nSat
    rdot_aj (jSat) = U_aj(:,jSat)' * ( C_e(:,:,jSat) * (v_ej(:,jSat) ...
        + Omega_ie * r_ej(:,jSat) ) - ( X(4:6) + Omega_ie * X(1:3) ) );
end
end