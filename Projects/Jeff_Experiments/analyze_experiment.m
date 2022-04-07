%% SCRIPT TO ANALYZE DATA:
% Clear up previous data:
close all
clear

% Pick the file:
% fileName = "CLVF_5_2.mat";
% fileName = "CLVF_5__stitched.mat";
% fileName = "CLVF_1_1.mat";
fileName = "Saved Data/SimulationData_2022_4_6_21_24/dataPacket_SIM.mat";
% fileName = "Saved Data/SimulationData_2022_4_1_22_52/dataPacket_SIM.mat";
% fileName = "Saved Data/SimulationData_2022_4_1_23_15/dataPacket_SIM.mat";
% fileName = "Saved Data/SimulationData_2022_4_1_23_19/dataPacket_SIM.mat"; ... from slow sim [1 deg/s]
% fileName = "Saved Data/SimulationData_2022_4_4_20_58/dataPacket_SIM.mat"; ... From the MPC at [1 deg/s]
% fileName = "Saved Data/SimulationData_2022_4_4_21_27/dataPacket_SIM.mat"; ... From retuned MPC at [1 deg/s]
% fileName = "Saved Data/SimulationData_2022_4_6_19_43/dataPacket_SIM.mat";


data = load(fileName);

try % Actual experiment data:
    data = data.rt_dataPacket;
catch % Simulation data:
    data = data.dataPacket;
end

% PARAMETERS I CANNOT CHANGE (physical characteristics of target).
d2r = pi/180;
theta_d             = 30*d2r;           % Angle of the docking cone.
d                   = [0.1611;0.4354;0];   % docking position.
d_norm              = sqrt(sum(d.^2));  % Norm of the docking position.
o_hat_prime         = [0;1;0];          % Orientation of the docking cone.
chaser_m = 12.3341;

left_cone_point = d + C3(theta_d)*o_hat_prime;
right_cone_point = d + C3(-theta_d)*o_hat_prime;

% Parameters for the radius for docking:
a = 0.7591; % radius for CLVF section.
a_prime = 0.2;
acceptable_radius = 0.05;

aTimesOVec =...
   [0.1600
    0.7420
    0.000];

% Create the circle for CLVF section:
theta = 0:0.01:2*pi;
clvf_circle_x = a*cos(theta);
clvf_circle_y = a*sin(theta);

% Create the circle for the LVF section:
lvf_circle_x = a_prime*cos(theta) + d(1);
lvf_circle_y = a_prime*sin(theta) + d(2);

% Create the acceptable circle:
acceptable_circle_x = acceptable_radius*cos(theta) + aTimesOVec(1);
acceptable_circle_y = acceptable_radius*sin(theta) + aTimesOVec(2);

%% EXTRACT DATA:
% Positions:
target_pos = data(:,5:7); % red
chaser_pos = data(:, 25:27); % black

% Velocities:
target_spd = data(:, 8:10); % red
chaser_spd = data(:, 28:30); % black

% Forces:
chaser_F = data(:,22:23);

% Time:
time = data(:,1);

% Clear out any indicies where we are all zeros:
bad_indicies = (target_pos(:,1) == 0);

% filter out positions that are pure zeros:
target_pos = target_pos(~bad_indicies,:);
chaser_pos = chaser_pos(~bad_indicies,:);
target_spd = target_spd(~bad_indicies,:);
chaser_spd = chaser_spd(~bad_indicies,:);
chaser_F = chaser_F(~bad_indicies, :);
time = time(~bad_indicies, :);

% How many points is this?
n_points = size(target_spd, 1);

%% CONVERT THE BLACK POSITION INTO RED'S BODY FIXED FRAME:
% Get the relative position and orientation:
relative_position = chaser_pos - target_pos;

% Convert relative positions into body-fixed:
relative_position_body = zeros(n_points, 2);

% solve the relative_position_body:
for i = 1:n_points
    % Get the rotation matrix:
    target_theta = target_pos(i, 3);
    C_BI = C3(target_theta); 
   
    % Rotate the relative position to the body_frame:
    pos_bf = C_BI * [relative_position(i,1); relative_position(i,2) ; 0];
    
    % Fill in the relative position matrix:
    relative_position_body(i,1) = pos_bf(1);
    relative_position_body(i,2) = pos_bf(2);
end

%% GET THE TOTAL FUEL USAGE FOR BLACK [DELTA V]:
acc_norm = zeros(n_points, 1);
delta_v = zeros(n_points, 1);

for i = 1:n_points
    acc_norm(i) = sqrt(sum(chaser_F(i,:).^2))/chaser_m;
    if i == 1
        delta_v(i) = acc_norm(i)*0.05; % Assuming running at 20Hz.
    else
        delta_v(i) = delta_v(i-1) + acc_norm(i)*0.05;
    end
end

%% PLOT:

% Create plot for the inertial frame:
figure
plot(target_pos(:,1), target_pos(:,2), 'r-');
grid on
hold on
plot(chaser_pos(:,1), chaser_pos(:,2), 'k-');
title("Table Position");
xlabel("X Position (m)");
ylabel("Y Position (m)");
axis equal;


% Create plot for the body frame:
figure
plot(relative_position_body(:,1), relative_position_body(:,2), 'k-','DisplayName','BF Position');
grid on;
hold on;
plot(d(1), d(2), 'rx', 'markersize',10,'DisplayName','Docking Port');
plot([d(1), d(1) + o_hat_prime(1)], [d(2), d(2) + o_hat_prime(2)], 'r--','DisplayName','Cone Direction');
plot(clvf_circle_x, clvf_circle_y, 'r-', "DisplayName","CLVF Circle");
plot(lvf_circle_x, lvf_circle_y, 'r-',"DisplayName","LVF Circle");
plot(acceptable_circle_x, acceptable_circle_y, 'r-', "DisplayName","Acceptable Radius");
plot([d(1), left_cone_point(1)],[d(2), left_cone_point(2)],'r-', "HandleVisibility","off");
plot([d(1), right_cone_point(1)],[d(2), right_cone_point(2)],'r-','DisplayName','Docking Cone');
title("Relative position");
xlabel("X Position (m)");
ylabel("Y Position (m)");
axis equal;
legend();

% Plot showing delta-v expendature over time:
figure
plot(time, delta_v);
grid on
hold on
xlabel("Time (s)")
ylabel("Delta V")
title("Delta V over time");
