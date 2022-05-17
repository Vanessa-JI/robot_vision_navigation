function integratedResults = IntegratedSolution(deadReckoningData, ...
    deadReckoningResults, Pseudo_range_rates, Pseudo_ranges, GNSSResults)
%INPUTS:
%   deadReckoningData:
%       Dead Reckoning data imported csv (matrix)
%   deadReckoningResults:
%       Results from deadReckoningSolution (struct)
%   GNSSResults:
%       Results from GNSS (struct)
%   tau: 
%       time interval (int)

%OUTPUTS:
%   integratedResults: Integrated solution of dead reckoning and GNSS
%       struct with fields:
%          times        Time in seconds – this must match the data files.
%          latitudes    Geodetic latitude in degrees
%          longitudes   Geodetic longitude in degrees
%          vel_N        North velocity in metres per second
%          vel_E        East velocity in metres per second
%          heading      Heading in degrees

    %Import useful constants
    Define_Constants; 

    times        = deadReckoningResults.times;
    drlatitudes  = deadReckoningResults.latitudes;
    drlongitudes = deadReckoningResults.longitudes;
    drvel_N      = deadReckoningResults.vel_N;
    drvel_E      = deadReckoningResults.vel_E;
    drheading    = deadReckoningResults.heading; %TODO convert to radians 
    gnsstimes      = GNSSResults.timeStamp;
    gnsslatitudes  = GNSSResults.latitude;
    gnsslongitudes = GNSSResults.longitude;
    gnssvel_N      = GNSSResults.velNorth;
    gnssvel_E      = GNSSResults.velEast;
    gnssheading    = GNSSResults.heading;
    gnssaltitude   = GNSSResults.altitude;
    
    drgyro = deadReckoningData(:,7) * deg_to_rad;
    
    nSat = length(Pseudo_range_rates(1, 2:end));

%Calculate diffference between systems , integrated kalman filter solution
%% TODO initialise x, P, h???
    [RN,RE] = Radii_of_curvature(drlatitudes(1));
    epochs = size(times,1);
    x = zeros(6,1);
    h = gnssaltitude(1);
    % Compute inital error covariance matrix
%     P = [sdDft^2 * ones(3,1);
%         sdOfs^2 * ones(3,1)];
% Assume initial velocity uncertainty 0.02, posiiton undecterinay 1
    P =  [0.02^2 * ones(3,1); 1; 1/(RN + h(1))^2; 1/((RE + h(1))^2 * cos(drlatitudes(1)))];
    P = diag(P);

    for i = 1:epochs
        %NED step 1: Transition matrix psi
        psi = eye(6);
        [RN,RE] = Radii_of_curvature(drlatitudes(i));
        psi(4,1) = tau / (RN + h);
        psi(5,2) = tau / ((RE + h) * cos(drlatitudes(i)));
        psi(6,3) = tau;
        %NED step 2: System Noise covariance matrix Q
        %PSD IN Define Constants? tau, h too?
        Q = [PSD * tau, 0, 0, 0.5 * (PSD * tau^2)/(RN +h), 0, 0;
            0, PSD * tau, 0, 0, 0.5 * (PSD * tau^2)/((RE + h)*cos(drlatitudes(i))), 0;
            0, 0, PSD * tau, 0, 0, -0.5 * PSD * tau^2;
            0.5 * (PSD * tau^2)/(RN +h), 0, 0, (PSD * tau^3)/(3*(RN +h)^2), 0, 0;
            0, 0.5 * (PSD * tau^2)/((RE + h)*cos(drlatitudes(i))), 0, 0, (PSD * tau^3)/(3*(cos(drlatitudes(i)))^2*(RN +h)^2), 0;
            0, 0, -0.5 * PSD * tau^2, 0, 0, (PSD*tau^3)/3];
        %NED step 3 & 4: Propagate State and Covariance x, P
        x = psi * x;
        P = psi * P * psi' + Q;
        %NED step 5: Calculate measurement matrix H
        %H = getMeaMat (nSat, x, 1);
        H = [zeros(3,3), -eye(3);
            -eye(3), zeros(3,3)];
        %% TODONED step 6: Measurement Noise Covariance R
%         R = [eye(3) * sdPseuRanMea^2, zeros(3);
%            zeros(3), eye(3) * sdPseuRanRateMea^2]; %Measurement noise comprises only the GNSS errors
        
        sdGr = 1;
        sdGv = 0.05;
        R = [ sdGr^2 /(RN + gnssaltitude(i))^2; sdGr^2/(RE+gnssaltitude(i))^2/(cos(drlatitudes(i)))^2; sdGr^2; sdGv*ones(3,1)];
        R = diag(R);
       
        %Step 7: Calculate Kalman Gain Matrix
        K = P * H' / (H * P * H' + R); 
        
        %% TODO Intermediate Step, find z (difference between DR & GNSSResults)
        z = [gnsslatitudes(i) - drlatitudes(i);
             gnsslongitudes(i) - drlongitudes(i);
             gnssaltitude(i);
             gnssvel_N(i) - drvel_N(i);
             gnssvel_E(i) - drvel_E(i);
             0 ];% down velocity assumed zero as 2D position tracked only.
            
             
             

        %NED Step 8: Measurement Innovation dz = z - H * x_pred
        dz = z - H * x;
        
        %Steps 9 & 10: Measurement Update x = x + K * dz; P = (I - K * H) * P
        x = x + K * dz;
        P = (eye(6) - K * H) * P;

        %% TODO Step 11:Results = difference between DR and states x
        latitudes(i) = drlatitudes(i) - x(4);
        longitudes(i) = drlongitudes(i) - x(5);
        vN(i) = drvel_N(i) - x(1);
        vE(i) = drvel_E(i) - x(2);
        

    end



%% Results
%Concatenate into struct and export to excel sheet 
    integratedResults.times = times';
    integratedResults.latitudes = latitudes;
    integratedResults.longitudes = longitudes; 
    integratedResults.vel_N = vN;
    integratedResults.vel_E = vE;
    %integratedResults.heading = heading;
    % odometry = timestamp, deadreckoning Latitude, deadreckoing
    %           Longitude, velocity North, velocity East, heading
    % No header row
    %Make struct into columns , MAY NOT BE NECESSARY!
    fns = fieldnames(integratedResults);
    for iField = 1:numel (fns)
        integratedResults.(fns{iField}) = integratedResults.(fns{iField})';
    end
    % Convert to table and export to excel file
    writetable(struct2table(integratedResults), 'Results/integratedResults.xlsx')
    
end 
