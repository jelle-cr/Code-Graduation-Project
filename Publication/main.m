%% Recreation of simulations paper Ming
close all
clear all
clc

N_a = 3;            % Number of trajectories to simulate (nominal + AC + APF-AC = 3)

% State Space
A = [0, 1;          
     1, 0];
B = [1, 0;
     0, 1];

% System Parameters
n = height(A);      % Number of states
m = width(B);       % Number of inputs
r_a = 0;            % Radius of agent 
r_o = 0.5;          % Radius of obstacle

x_0 = [0, 0, 0;     % Initial positions
       0, 0, 0];
x_d = [3;           % Desired position
       5];
x_o = [1, 2.5, 4;   % Obstacle positions
       1.5, 3, 4.2];
N_o = width(x_o);   % Number of obstacles

% Potential Field Parameters
k_att = 1;          % Attractive potential gain
k_rep = 1;          % Repulsive potential gain
rho_0 = 0.5;        % Repulsive potential range

% Simulation Time
t_end = 5;          % Simulate for 5 seconds
t_step = 0.001;     % Controller frequency 1000Hz
t = 0:t_step:t_end; % Simulation time

% Save necessary parameters
save('Data/Parameters.mat', 'A', 'B', 'n', 'm', 'r_a', 'N_a', 'r_o', 'N_o', ...
                            'k_att', 'k_rep', 'rho_0', 'x_0', 'x_d', 'x_o', ...
                            't_end', 't_step');
fprintf('Saved Parameters\n');

% Simulate
x = reshape(Functions.ode4(@Functions.odefcn, t, reshape(x_0, [], 1)).', n, N_a, length(t)); 

%% Plotting
rangeX = [-1; 6];
rangeY = [0; 6];
Functions.plot_trajectory(rangeX, rangeY, x, x_0, x_d, x_o, r_o, N_o, rho_0);