%% SCRIPT TO ANALYZE DATA:
% Clear up previous data:
close all
clear

% Pick the file:
fileName = "CLVF_9_1.mat";
data = load(fileName);
data = data.rt_dataPacket;

% PARAMETERS I CANNOT CHANGE (physical characteristics of target).
d2r = pi/180;
theta_d             = 30*d2r;           % Angle of the docking cone.
d                   = [0.16;0.542;0];   % docking position.
d_norm              = sqrt(sum(d.^2));  % Norm of the docking position.
o_hat_prime         = [0;1;0];          % Orientation of the docking cone.

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

% Clear out any indicies where we are all zeros:
bad_indicies = (target_pos(:,1) == 0);

% filter out positions that are pure zeros:
target_pos = target_pos(~bad_indicies,:);
chaser_pos = chaser_pos(~bad_indicies,:);
target_spd = target_spd(~bad_indicies,:);
chaser_spd = chaser_spd(~bad_indicies,:);

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
title("Relative position");
xlabel("X Position (m)");
ylabel("Y Position (m)");
axis equal;
legend();
