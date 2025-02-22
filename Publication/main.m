%% Euler Lagrangian System
close all
clear all
clc

% System Parameters
A = [0, 1;          
     1, 0];
B = [1, 0;
     0, 1];

n = height(A);      % Number of states
m = width(B);       % Number of inputs
r_a = 0;            % Radius of agent 
r_o = 0.5;          % Radius of obstacle

x_0 = [-4;                  % Initial position
       -2];
x_d = [4;                   % Desired position
       2];
x_o = [-2,   -0.5,  2;      % Obstacle positions
       -1.25, 1.25, 0.75];
% x_o = [-2;      % Obstacle position
%        -1.25];
N_o = width(x_o);

% Potential Field Parameters
k_att = 1;          % Attractive potential gain
k_rep = 0.1;        % Repulsive potential gain
h_0 = 1;            % Repulsive potential range

% Simulation Time
t_end = 10;         % Simulate for 10 seconds
t_step = 0.01;     
t = 0:t_step:t_end; % simulation time

% Save necessary parameters
save('Data/Parameters.mat', 'A', 'B', 'n', 'm', 'r_a', 'r_o', 'N_o', ...
                            'k_att', 'k_rep', 'h_0', 'x_0', 'x_d', 'x_o', ...
                            't_end', 't_step');
fprintf('Saved Parameters\n');

x = Functions.ode4(@Functions.odefcn, t, reshape(x_0, [], 1)).'; 

%% Plotting
rangeX = [-5; 5];
rangeY = [-3; 3];
Functions.plot_trajectory(rangeX, rangeY, x, x_0, x_d, x_o, r_o, N_o);




%% Euler Lagrangian System
% close all
% clear all
% clc
% 
% N_a = 2;
% 
% % System Parameters
% A = [0, 1;          
%      1, 0];
% B = [1, 0;
%      0, 1];
% 
% n = height(A);      % Number of states
% m = width(B);       % Number of inputs
% r_a = 0;            % Radius of agent 
% r_o = 0.5;          % Radius of obstacle
% 
% x_0 = [-4, -1;              % Initial positions
%        -2, -1];
% x_d = [4;                   % Desired position
%        2];
% x_o = [-2,   -0.5,  2;      % Obstacle positions
%        -1.25, 1.25, 0.75];
% % x_o = [-2;      % Obstacle position
% %        -1.25];
% N_o = width(x_o);
% 
% % Potential Field Parameters
% k_att = 1;          % Attractive potential gain
% k_rep = 1;          % Repulsive potential gain
% h_0 = 1;            % Repulsive potential range
% 
% % Simulation Time
% t_end = 10;         % Simulate for 10 seconds
% t_step = 0.01;     
% t = 0:t_step:t_end; % simulation time
% 
% % Save necessary parameters
% save('Data/Parameters.mat', 'A', 'B', 'n', 'm', 'r_a', 'N_a', 'r_o', 'N_o', ...
%                             'k_att', 'k_rep', 'h_0', 'x_0', 'x_d', 'x_o', ...
%                             't_end', 't_step');
% fprintf('Saved Parameters\n');
% 
% x = reshape(Functions.ode4(@Functions.odefcn, t, reshape(x_0, [], 1)).', n, N_a, length(t)); 
% 
% %% Plotting
% rangeX = [-5; 5];
% rangeY = [-3; 3];
% Functions.plot_trajectory(rangeX, rangeY, x, x_0, x_d, x_o, r_o, N_o);