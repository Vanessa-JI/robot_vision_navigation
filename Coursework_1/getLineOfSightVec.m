function U_aj = getLineOfSightVec (nSat, C_e, r_ej, r_aj, X)
% Compute satellite-antenna line-of-sight unit vector
%
% Inputs:
%   X           8x1 state vector
%   nSat        total number of satellites
%   C_e         Sagnac effect compensation matrix
%   r_ej        satellite-earth position vector
%   r_aj        predicted satellite-antenna range vector
%
% Ouputs:
%   U_aj        satellite-antenna line-of-sight unit vector

r_ea = X (1:3);
U_aj = zeros(3,nSat);
for jSat = 1: nSat
    U_aj(:,jSat) = (C_e(:,:,jSat) * r_ej (:,jSat) - r_ea) / r_aj(jSat);
end
end