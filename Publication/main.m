%% Euler Lagrangian System
close all
clear all
clc

N_a = 3;

% System Parameters
A = [0, 1;          
     1, 0];
B = [1, 0;
     0, 1];

n = height(A);      % Number of states
m = width(B);       % Number of inputs
r_a = 0;            % Radius of agent 
r_o = 0.5;          % Radius of obstacle

x_0 = [0, 0, 0;          % Initial positions
       0, 0, 0];
x_d = [3;                   % Desired position
       5];
x_o = [1, 2.5, 4;      % Obstacle positions
       1.5, 3, 4.2];
% x_o = [-2;      % Obstacle position
%        -1.25];
N_o = width(x_o);

% Potential Field Parameters
k_att = 1;          % Attractive potential gain
k_rep = 1;        % Repulsive potential gain
rho_0 = 0.5;            % Repulsive potential range

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
% x_0 = [-4, -4, -4;          % Initial positions
%        -2, -2, -2];
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
% k_rep = 0.1;        % Repulsive potential gain
% rho_0 = 1;            % Repulsive potential range

% Simulation Time
t_end = 5;         % Simulate for 10 seconds
t_step = 0.001;     
t = 0:t_step:t_end; % simulation time

% Save necessary parameters
save('Data/Parameters.mat', 'A', 'B', 'n', 'm', 'r_a', 'N_a', 'r_o', 'N_o', ...
                            'k_att', 'k_rep', 'rho_0', 'x_0', 'x_d', 'x_o', ...
                            't_end', 't_step');
fprintf('Saved Parameters\n');

x = reshape(Functions.ode4(@Functions.odefcn, t, reshape(x_0, [], 1)).', n, N_a, length(t)); 

%% Plotting
rangeX = [-1; 6];
rangeY = [0; 6];
Functions.plot_trajectory(rangeX, rangeY, x, x_0, x_d, x_o, r_o, N_o, rho_0);