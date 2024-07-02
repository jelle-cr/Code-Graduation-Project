close all
% clear all
clc
warning on; 

%% State space
A = [0, 0;
     0, 0];
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
% APF = false;  % Use CBF regions
rho_0 = 0.5;
K_att = 1;
K_rep = 0.001;

%% Initial positions
X_0 = [0,  2, 3, -1;
       0, -1, 5,  2];

%% Desired positions
% X_d = reshape(Functions.calculate_desired_trajectories(t), n, N_a);
X_d = [3, 1, 0, 4; 
       5, 6, 1, 4];

%% Obstacles
X_o = [1,   2.5, 4;
       1.5, 3,   4.2]; % Location of obstacles   
r_o = 0.4;             % Radius of obstacle    
X_o = [];

save('Data/Parameters.mat', 'A', 'B', 'n', 'm', 'N_a', 'r_a', 'u_max', 'X_d', 'APF', 'rho_0', 'K_att', 'K_rep', 'X_o', 'r_o');

%% Simulation
t_end = 5;
t_step = 0.01;
t = 0:t_step:t_end;  % simulation time

[X] = reshape(Functions.ode4(@Functions.odefcn, t, reshape(X_0, [], 1)).', n, N_a, length(t)); % Column vector

%% Plot results
t_stop = 0.8;
Functions.plot_real_time_trajectories(X, t, t_stop);

