clear; clc; close all;

%% The Tesing Field
% Our vehicle will run with a velocity on a round track with a designated radius

%%% The track
R_track = 500; % 1000 meters

%%% Simulation configuration
simTime = 120; % seconds
delta_t = 0.01; % time
t = 0:delta_t:simTime;
simulationSpeed = 3; % play the animation in 2x speed

%%% The vehicle
V_car = 28; % m/s, instantaneous velocity, aka tangent velocity
T_car = 2*pi*R_track/V_car; % time spent to drive one round
F_car = 1/T_car;
% time; x position; y position; pose (angle between X_car & X_world)
Pos_car = [t; R_track*cos(2*pi*F_car.*t); R_track*sin(2*pi*F_car.*t); 2*pi*F_car.*t + pi/2];

%% Plotting
figure('Position', [10 100 800 800]);
% Plot the true trajectory
plot(Pos_car(2,:), Pos_car(3,:), '--','LineWidth', 1); hold on;

%%% Plot Configurations
axis equal;
xlim([0 R_track*1.1]); ylim([0 R_track*1.1]);
grid on; grid minor;
xlabel('x [m]', 'Interpreter','latex','FontSize',12);
ylabel('y [m]', 'Interpreter','latex','FontSize',12);
title('Simulation', 'Interpreter','latex','FontSize',14);
% legend('True Trajectory', ...
%        'True Position', ...
%        'True Direction', ...
%        'Interpreter','latex','FontSize',12, 'Location', 'southwest');

%%% Animation
% Position
Pos_car_plot = [Pos_car(2, 1); Pos_car(3, 1)];
h_car = plot(Pos_car_plot(1), Pos_car_plot(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
% Pose
arrowLength = 40;
arrowEnd = Pos_car_plot + arrowLength * [cos(Pos_car(4, 1)); sin(Pos_car(4, 1))];
h_arrow = quiver(Pos_car_plot(1), Pos_car_plot(2), ...
                 arrowLength*cos(Pos_car(4, 1)), arrowLength*sin(Pos_car(4, 1)), ...
                 'LineWidth', 2, 'MaxHeadSize', 4, 'color',[0 0 1]);
% Drawing
for i = 2:length(t)-1
    Pos_car_plot = [Pos_car(2, i); Pos_car(3, i)];
    arrowEnd = Pos_car_plot + arrowLength * [cos(Pos_car(4, i)); sin(Pos_car(4, i))];
    
    % Update car position and arrow direction
    set(h_car, 'XData', Pos_car_plot(1), 'YData', Pos_car_plot(2));
    set(h_arrow, 'XData', Pos_car_plot(1), 'YData', Pos_car_plot(2), ...
        'UData', arrowEnd(1) - Pos_car_plot(1), 'VData', arrowEnd(2) - Pos_car_plot(2));
    
    pause(delta_t/simulationSpeed);
    drawnow;
end

