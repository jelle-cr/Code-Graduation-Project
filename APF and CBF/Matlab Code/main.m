close all
clear all
clc
warning on; 

global u_save

overrideNominalTrajectory = false;

% State space
A = [0 1; 1 0];
B = eye(2);

n = height(A);
m = width(B);

% Quadcopter parameters
num_axes = 2;            % Number of axes (x,y,z)
N_a = 2;                 % Number of agents
% M = 0.01;                % Mass [kg]
% d = 0.1;                 % Damping coefficient [Ns/m]
r_a = 0.05;              % Radius of agent [m]
u_max = 1;               % Maximum control force [N]

% Initial positions
agent_spacing = 0.5;     % Spacing of formation circle agents (0.3 or 0.5 works well)
X_0 = Functions.generate_circular_initial_positions(N_a, r_a, agent_spacing);   % Column vector
if overrideNominalTrajectory
    X_0 = [-0.4; 0; 0.4; 0];
end

% Nominal trajectories
use_V_ref = false;       % Determines whether or not to use reference velocity in CLF nominal control calculation
origin_max = 0.1;
origin_min = -origin_max;
A_min = 0.1;
A_max = 0.2;
f_min = 5;
f_max = 10;
phi_max = pi;
phi_min = -pi;
origin_rand = (origin_max-origin_min)*rand(2,N_a)+origin_min;
A_rand = (A_max-A_min)*rand(1,N_a)+A_min;
f_rand = (f_max-f_min)*rand(1,N_a)+f_min;
phi_rand = (phi_max-phi_min)*rand(1,N_a)+phi_min;
sign_rand = sign(randi([0, 1], num_axes, N_a) - 0.5);

% load('Data/FixedTrajectoryParameters.mat');    % Uncomment to use specific saved nominal trajectories
save('Data/TrajectoryParameters.mat', 'origin_rand', 'A_rand', 'f_rand', 'phi_rand', 'sign_rand', 'use_V_ref', 'N_a');
% save('Data/FixedTrajectoryParameters.mat', 'origin_rand', 'A_rand', 'f_rand', 'phi_rand', 'sign_rand', 'use_V_ref', 'N_a');

% APF parameters
K_att = 5000;
K_rep = 0.00001;
rho_0 = 2*r_a;

save('Data/Parameters.mat', 'num_axes', 'n', 'm', 'N_a', 'r_a', 'u_max', 'X_0', 'K_att', 'K_rep', 'rho_0', 'overrideNominalTrajectory');

% Time vector
t_end = 2;
t_step = 0.001;
t_span = 0:t_step:t_end;  % simulation time
num_steps = length(t_span);

[X] = reshape(Functions.ode4(@Functions.odefcn, t_span, X_0).', [], 1); % Column vector

u_save = reshape(u_save, m, N_a, length(u_save));

X_d = [];
for t = 0:t_step:t_end
    X_d = [X_d; Functions.calculate_desired_trajectories(t, overrideNominalTrajectory, X_0)];
end

%% Plot results
close all;
fontsize = 16;
Functions.plot_real_time_trajectories(X, X_d, t_end, t_step, num_steps, fontsize);

