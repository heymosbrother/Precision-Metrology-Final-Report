clear; clc; close all;

%% The Tesing Field
% Our vehicle will run with a velocity on a round track with a designated radius

%%% The track
R_track = 500; % 1000 meters

%%% Simulation configuration
simTime = 60; % seconds
dt = 0.1; % time
t = 0:dt:simTime;
simulationSpeed = 6; % play the animation in speed X

%%% The vehicle
% Reference vehicle
V_car = 14; % m/s, instantaneous velocity, aka tangent velocity
T_car = 2*pi*R_track/V_car; % time spent to drive one round
F_car = 1/T_car;
% time; x position; y position; pose (angle between X_car & X_world)
Pos_car = [t; R_track*cos(2*pi*F_car.*t); R_track*sin(2*pi*F_car.*t); 2*pi*F_car.*t + pi/2];

% Inner vehicle
R_in = 10;
V_car_in = 15; % tangential velocity
T_car_in = 2*pi*(R_track-R_in)/V_car;
F_car_in = 1/T_car_in;
Pos_car_inner = [t; (R_track-R_in)*cos(2*pi*F_car_in.*t); (R_track-R_in)*sin(2*pi*F_car_in.*t)];

% --------------------------------------------------------------------------------------------------

%% Plotting
figure('Position', [10 100 800 800]);
% Plot the true trajectory
plot(Pos_car(2,:), Pos_car(3,:), '--','LineWidth', 1); hold on; % subject vehicle
plot(Pos_car_inner(2, :), Pos_car_inner(3, :), '--', 'LineWidth', 0.5); hold on;

%%% Animation
% Position
Pos_car_plot = [Pos_car(2, 1); Pos_car(3, 1)];
h_car = plot(Pos_car_plot(1), Pos_car_plot(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0, 0.4470, 0.7410]);
% Pose
arrowLength = 40;
arrowEnd = Pos_car_plot + arrowLength * [cos(Pos_car(4, 1)); sin(Pos_car(4, 1))];
h_arrow = quiver(Pos_car_plot(1), Pos_car_plot(2), ...
                 arrowLength*cos(Pos_car(4, 1)), arrowLength*sin(Pos_car(4, 1)), ...
                 'LineWidth', 2, 'MaxHeadSize', 4, 'color',[0, 0.4470, 0.7410]);

% Plot Configurations
axis equal;
xlim([0 R_track*1.1]); ylim([0 R_track*1.1]);
grid on; grid minor;
xlabel('x [m]', 'Interpreter','latex','FontSize',12);
ylabel('y [m]', 'Interpreter','latex','FontSize',12);
title('Simulation', 'Interpreter','latex','FontSize',14);


estimateRecordX = zeros(size(t));    
measureRecordX = zeros(size(t));
truePosX = zeros(size(t));
% Drawing
for i = 2:length(t)-1
    Pos_car_plot = [Pos_car(2, i); Pos_car(3, i)];
    truePosX = Pos_car_plot(1);
    arrowEnd = Pos_car_plot + arrowLength * [cos(Pos_car(4, i)); sin(Pos_car(4, i))];
    
    %%% Update car position and arrow direction
    set(h_car, 'XData', Pos_car_plot(1), 'YData', Pos_car_plot(2));
    set(h_arrow, 'XData', Pos_car_plot(1), 'YData', Pos_car_plot(2), ...
        'UData', arrowEnd(1) - Pos_car_plot(1), 'VData', arrowEnd(2) - Pos_car_plot(2));
    
    % Mark measured position value
    Pos_inner_plot = [Pos_car_inner(2, i); Pos_car_inner(3, i)];
    lidar_measured = lidar(Pos_inner_plot);
    
    if rem(i, 10) == 0
        plot(lidar_measured(1), lidar_measured(2), 'rx'); hold on;
    end
    
    % Mark estimated postition value
    Pos_inner_estimated = kalmanfilter([lidar_measured(1); lidar_measured(2)]);
    if rem(i, 2) == 0
        plot(Pos_inner_estimated(1), Pos_inner_estimated(2), 'gx'); hold on;
    end
    
    % Record the measured estimated data
    estimateRecordX(i) = Pos_inner_estimated(1);
    measureRecordX(i) = lidar_measured(1);
    truePosX = Pos_car(2, i);
%    pause(dt/simulationSpeed);
    drawnow;
end

% plot the analysis graph
figure('Position', [500 100 800 800]);
plot(t, Pos_car_inner(2, :), '-','LineWidth', 1); hold on;
plot(t, estimateRecordX, 'LineWidth', 1.5); hold on;
plot(t, measureRecordX, '-', 'LineWidth', 1); hold on;
grid on; grid minor;
legend('true $x$ value', 'estimated $x$ value', 'measured $x$ value', 'Interpreter','latex','FontSize',12, 'Location', 'southeast');
ylabel('$x$ position','Interpreter','latex','FontSize',12)
xlabel('Time [sec]','Interpreter','latex','FontSize',12)
title('Kalman filter analysis', 'Interpreter','latex','FontSize',12);
xlim([0 20]);

function gps_measured = gpsdata(truePos)
    x_m = truePos(1) + random('Normal', 0, 5);
    y_m = truePos(2) + random('Normal', 0, 5);
    gps_measured = [x_m y_m];
end

function lidar_measured = lidar(truePos)
    x_m = truePos + random('Normal', 0, 2/0.49438);
    y_m = truePos + random('Normal', 0, 2/0.49438);
    lidar_measured = [x_m, y_m];
end

function y = kalmanfilter(z) 

dt=1;
% Initialize state transition matrix
A=[ 1 0 dt 0 0 0;...     % [x  ]            
       0 1 0 dt 0 0;...     % [y  ]
       0 0 1 0 dt 0;...     % [Vx]
       0 0 0 1 0 dt;...     % [Vy]
       0 0 0 0 1 0 ;...     % [Ax]
       0 0 0 0 0 1 ];       % [Ay]

H = [ 1 0 0 0 0 0; 0 1 0 0 0 0 ];    % Initialize measurement matrix

Q = eye(6);

R = 1000 * eye(2);


persistent x_est p_est                % Initial state conditions
if isempty(x_est)
    x_est = zeros(6, 1);             % x_est=[x,y,Vx,Vy,Ax,Ay]'
    p_est = zeros(6, 6);
end

% Predicted state and covariance
x_prd = A * x_est;
p_prd = A * p_est * A' + Q;
% Estimation
S = H * p_prd' * H' + R;
B = H * p_prd';
klm_gain = (S \ B)';
% Estimated state and covariance
x_est = x_prd + klm_gain * (z - H * x_prd);
p_est = p_prd - klm_gain * H * p_prd;
% Compute the estimated measurements
y = H * x_est;
end                % of the function