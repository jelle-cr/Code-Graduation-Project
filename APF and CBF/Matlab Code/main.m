close all
clear all
clc
warning on; 

model = 'singleIntegrator';
% model = 'doubleIntegrator';

%% Quadcopter parameters
d = 0.01;
M = 0.1;

%% State space
% This input method only allows linear systems, change later
if strcmp(model, 'singleIntegrator')
    A = [0, 1;
         1, 0];
    B = [1, 0;
         0, 1];
end
if strcmp(model, 'doubleIntegrator')
    A = [0, 0, 1, 0;
         0, 0, 0, 1;
         0, 0, 0, 0;
         0, 0, 0, 0];
    B = [0, 0;
         0, 0;
         1, 0;
         0, 1];
end

%% Agent Parameters
n = height(A);
m = width(B);
N_a = 4;
r_a = 0.5;
u_max = 10;

%% Controller parameters
APF = true;     % Use APF regions
APF = false;    % Use CBF regions
rho_0 = 0.5;
K_att = 1;
K_rep = 0.001;

%% Simulation Parameters
t_end = 5;
t_step = 0.01;
t = 0:t_step:t_end;  % simulation time

%% Obstacles
distanceRange = [-3, 3];
N_o = 6;               % Number of obstacles
p_o = rand(2,N_o)*(distanceRange(2)-distanceRange(1))+distanceRange(1);
                       % Position of obstacles   
r_o = 0.4;             % Radius of obstacle    

%% Initial states
formationDistance = 3*r_a;
p_0 = Functions.generate_initial_positions(model, N_a, r_a, formationDistance, p_o, r_o);

%% Desired states
numWaypoints = 1;    % =1 results in static goal position   
p_d = Functions.generate_desired_positions(n, N_a, t, numWaypoints, distanceRange);

%% Save necessary parameters
% load('Data/Parameters.mat');
save('Data/Parameters.mat', 'model', 'A', 'B', 'n', 'm', 'N_a', 'r_a', 'u_max', 't_step', 'p_0', 'p_d', 'APF', 'rho_0', 'K_att', 'K_rep', 'N_o', 'p_o', 'r_o');

%% Simulate
[p] = reshape(Functions.ode4(@Functions.odefcn, t, reshape(p_0, [], 1)).', n, N_a, length(t)); % Column vector

%% Plot results
t_stop = t_end;
Functions.plot_real_time_trajectories(p, t, t_stop);

