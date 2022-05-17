function [r_aj, C_e, r_ej, v_ej] = predictRanges(RawData, X, nSat, omega_ie, c, t)
% Predict ranges from antenna to each satellite
%
% Inputs:
%   RawData     struct containing raw data
%   X           8x1 state vector
%   nSat        total number of satellites
%   omega_ie    Earth rotation rate in rad/s
%   c           Speed of light in m/s
%   t           current runtime
%
% Outputs:
%   r_aj        predicted satellite-antenna range vector
%   C_e         Sagnac effect compensation matrix
%   r_ej        satellite-earth position vector
%   v_ej        satellite-earth velocity vector


% initialise antenna-earth position vector, r_ea
r_ea = X(1:3);
% initialise satellite-antenna range, r_aj
r_aj = zeros(1, nSat);
% initialise satellite-earth position vector, r_ej
r_ej = zeros(3, nSat);
% initialise satellite-earth velocity vector, v_ej
v_ej = zeros(3, nSat);
% initialise Sagnac effect compensation matrix
C_e = repmat(eye(3),1,1,nSat);

for jSat = 1: nSat
    % get satellite-to-earth position (r_ej) & velocity (v_ej)
    [r_ej(:,jSat), v_ej(:,jSat)] = Satellite_position_and_velocity (t, RawData.satNos(jSat));
    % Initialise previous satellite-antenna range record, r_aj_prev
    r_aj_prev = 10;
    while abs(r_aj(jSat) - r_aj_prev) > 0.001
        r_aj_prev = r_aj(jSat);
        r_aj(jSat) = sqrt( (C_e(:,:,jSat) * r_ej(:,jSat) - r_ea)' * (C_e(:,:,jSat) * r_ej(:,jSat) - r_ea) );
        C_e(:,:,jSat) = [1, omega_ie * r_aj(jSat) / c, 0;
            -omega_ie * r_aj(jSat) / c, 1, 0;
            0, 0, 1];
    end
end
end