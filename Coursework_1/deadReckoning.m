function deadReckoningResults = deadReckoning(dead_reckoning_data, GNSSResults)
%INPUTS:
%       init_longitude:
%           GNSS  initial longitude result in radians
%       init_latitude:
%           GNSS initial latitude results in radians
%       init_altitude:
%           GNSS initial height result
%
%OUTPUTS:
%       deadReckoningResults    DR results struct with fields:
%          times        Time in seconds – this must match the data files.
%          latitudes    Geodetic latitude in degrees
%          longitudes   Geodetic longitude in degrees
%          vel_N        North velocity in metres per second
%          vel_E        East velocity in metres per second
%          heading      Heading in degrees

% Everything to 15 decimal places using format long
format long;
%Load in constants
Define_Constants;

%Load in GNSSResults
init_longitude = GNSSResults.longitude(1);
init_latitude = GNSSResults.latitude(1);
init_altitude = 0; %Set initial altitude as zero
%Initialise variables
times = dead_reckoning_data(:,1);
%rear wheels are driving wheels, thus 3 and 4
wheel_speed_3 = dead_reckoning_data(:,4);
wheel_speed_4 = dead_reckoning_data(:,5);
average_wheel_velocity = 0.5.*(wheel_speed_3 + wheel_speed_4);
heading = dead_reckoning_data(:,7) * deg_to_rad;

% Calculate longitude and latitudes
% Initialise empty matrix
vel_N = zeros(size(average_wheel_velocity));
vel_E = zeros(size(average_wheel_velocity));
% Initilaise first value of Velocity East, North
vel_N(1) = 0.5 * (cos(heading(1)) + cos(heading(1))) * average_wheel_velocity(1);
vel_E(1) = 0.5 * (sin(heading(1)) + sin(heading(1))) * average_wheel_velocity(1);

for i = 2:size(average_wheel_velocity,1)
    vel_N(i) = 0.5 * (cos(heading(i)) + cos(heading(i-1))) * average_wheel_velocity(i);
    vel_E(i) = 0.5 * (sin(heading(i)) + sin(heading(i-1))) * average_wheel_velocity(i);
end

% Initialise empty matrix
latitudes = zeros(size(times));
longitudes = zeros(size(times));
[Rad_N, Rad_E] = Radii_of_curvature(init_latitude);
latitudes(1) = init_latitude + vel_N(1) * (times(1) - 0) / (Rad_N + init_altitude);
longitudes(1) = init_longitude + vel_E(1) * (times(1) - 0) / ((Rad_E + init_altitude) * cos(latitudes(1)));
for i = 2:size(times,1)
    [Rad_N, Rad_E] = Radii_of_curvature(latitudes(i-1));
    latitudes(i) = latitudes(i-1) + vel_N(i) * (times(i)-times(i-1)) / (Rad_N + init_altitude);
    longitudes(i) = longitudes(i-1) + vel_E(i) * (times(i)-times(i-1)) / ((Rad_E + init_altitude) * cos(latitudes(i)));
end

% TODO Calculate damped instant veloctiy

v0 = average_wheel_velocity(1);
vN = zeros(size(vel_N));
vE = zeros(size(vel_E));
psi0 = heading(1);
psi = heading;
vN(1) = v0 * cos(psi0);
vE(1) = v0 * sin(psi0);
%L3AS58
for i = 2:size(vel_N,1)
    vN(i) = cos(psi(i)) * vel_N(i) - sin(psi(i)) * vN(i-1); 
    vE(i) = sin(psi(i)) * vel_E(i) + cos(psi(i)) *vE(i-1);
end
%% TODO CONVERT TO RADIANS
%TODO Concatenate into struct and export to excel sheet
deadReckoningResults.times = times;
deadReckoningResults.latitudes = latitudes;
deadReckoningResults.longitudes = longitudes;
deadReckoningResults.vel_N = vN;
deadReckoningResults.vel_E = vE;
deadReckoningResults.heading = heading;
% odometry = timestamp, deadreckoning Latitude, deadreckoing
%           Longitude, velocity North, velocity East, heading
% No header row
% Convert to table and export to excel file

writetable(struct2table(deadReckoningResults), 'Results/deadReckoningResults.xlsx')


end