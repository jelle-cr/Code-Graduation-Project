% Single integrator system
clear all
clc

dynamics = 'Single Integrator';

%% Simulation parameters
N_a = 1;            % Number of trajectories to simulate
N_o = 3;            % Number of obstacles

A = [0, 0;          % State space
     0, 0];
B = [1, 0;
     0, 1];

n = height(A);      % Number of states
m = width(B);       % Number of inputs
u_max = 30;         % Maximum control input in 1 direction
r_a = 0;            % Radius of agent 
r_o = 0.75;          % Radius of obstacle

% Potential field parameters
k_att = 1;          % Attractive potential gain
k_rep = 0.01;          % Repulsive potential gain
rho_0 = 1.5;        % Repulsive potential range

% x_0 = [-2.5;        % Initial position
%        -2.5];
x_0 = [-2.5;     % Initial positions
       -1.5];
x_d = [2;           % Desired position
       1];
x_o = [-1, -0.25, 1.5;           % Obstacle positions
       -1, 1, -0.25];

% Simulation time
t_end = 5;
t_step = 0.01;
t = 0:t_step:t_end;  % simulation time
num_steps = length(t);

% Save necessary parameters
save('Data/Parameters.mat', 'N_a', 'N_o', 'A', 'B', 'n', 'm', 'u_max', 'r_a', 'r_o', ...
                            'k_att', 'k_rep', 'rho_0', 'x_0', 'x_d', 'x_o', ...
                            't_end', 't_step');

fprintf('Saved System Parameters\n');

%% Simulate
tic
x = reshape(Functions.ode4(@Functions.odefcn, t, reshape(x_0, [], 1)).', n, N_a, length(t)); % Column vector
fprintf('Simulation Done\n');
toc

% Save complete state and control input 
load('Data/SimulationData.mat');    % Loads u that was saved
save('Data/SimulationData.mat', 'x', 'u');

%% Plot results
close all

rangeX = [-3; 3];
rangeY = [-2; 2];
Functions.plot_stationary(rangeX, rangeY, t_stop);

