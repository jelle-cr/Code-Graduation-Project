close all
clear all
clc
warning on; 

model = 'singleIntegrator';
model = 'doubleIntegrator';

%% State space
A = [0, 0;
     0, 0];  % This input method only allows linear systems, change later
B = [1, 0;
     0, 1];

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
N_o = 10;               % Number of obstacles
p_o = rand(2,N_o)*(distanceRange(2)-distanceRange(1))+distanceRange(1);
                       % Position of obstacles   
r_o = 0.4;             % Radius of obstacle    

%% Initial states
formationDistance = 3*r_a;
p_0 = Functions.generate_initial_positions(N_a, r_a, formationDistance, p_o, r_o);

%% Desired states
numWaypoints = 3;
p_d = Functions.generate_desired_positions(n, N_a, t, numWaypoints, distanceRange);
% X_d = [3, 1, 0, 4; 
%        5, 6, 1, 4];

%% Save necessary parameters
save('Data/Parameters.mat', 'A', 'B', 'n', 'm', 'N_a', 'r_a', 'u_max', 't_step', 'p_d', 'APF', 'rho_0', 'K_att', 'K_rep', 'N_o', 'p_o', 'r_o');

%% Simulate
[p] = reshape(Functions.ode4(@Functions.odefcn, t, reshape(p_0, [], 1)).', n, N_a, length(t)); % Column vector

%% Plot results
t_stop = t_end;
Functions.plot_real_time_trajectories(p, t, t_stop);

