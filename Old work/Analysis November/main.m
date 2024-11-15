% Cascaded controller for Double Integrator system
% close all
clear all
clc

range = 3;
num_steps = 500;
x = linspace(-range, range, num_steps);
y = linspace(-range, range, num_steps);

%% Simulation parameters
N_a = 1;            % Number of trajectories to simulate
N_o = 1;            % Number of obstacles

n = 4;              % Number of states
m = 2;              % Number of inputs
u_max = 300;         % Maximum control input in 1 direction
r_a = 0.5;          % Radius of agent 

% Potential field parameters
k_att_p = 5;        % Attractive position gain
k_att_v = 1;        % Attractive velocity gain
k_rep = 0.001;      % Repulsive gain
rho_0 = 6;        % Repulsive potential range
r_o = 0.6;          % Radius of obstacle

% Obstacle states, desired states, and initial states
x_o = [-0.1; -0.4; 0; 0];
x_d = [2; 1; 0; 0];
x_0 = [-2.5; -2.5; 0; 0];

% Simulation time
t_end = 5;
t_step = 0.01;
t = 0:t_step:t_end;  % simulation time

% Save necessary parameters
% load('Data/Parameters.mat');
save('Data/Parameters.mat', 'n', 'm', 'N_a', 'r_a', 'u_max', 't_step', 'x_0', 'x_d', 'k_rep', 'rho_0', 'N_o', 'x_o', 'r_o');

fprintf('Saved System Parameters\n');

%% Simulate
tic
[x] = reshape(Functions.ode4(@Functions.odefcn, t, reshape(x_0, [], 1)).', n, length(t)); % Column vector
p = x(1:2,:);
fprintf('Simulation Done\n');
toc

%% Plot results
close all
delay = 0;

t_stop = t_end;
Functions.plot_real_time_trajectories(p, range, t_stop, t_step, delay);

