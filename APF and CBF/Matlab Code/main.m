close all
clear all
clc
warning on; 

global u_save

overrideNominalTrajectory = false;

% Quadcopter parameters
num_axes = 2;            % Number of axes (x,y,z)
N_a = 2;                 % Number of agents
M = 0.01;                % Mass [kg]
d = 0.1;                 % Damping coefficient [Ns/m]
r_a = 0.5;              % Radius of agent [m]
u_max = 1;               % Maximum control force [N]

% State space
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, 0, -d/M, 0;
     0, 0, 0, -d/M];
B = [0, 0;
     0, 0;
     1/M, 0;
     0, 1/M];

A = [0, 1;
     1, 0];
B = [1, 0;
     0, 1];

n = height(A);
m = width(B);

X_0 = [0, 1;0, 1];

% Time vector
t_end = 10;
t_step = 0.01;
t_span = 0:t_step:t_end;  % simulation time
num_steps = length(t_span);

[X] = reshape(Functions.ode4(@Functions.odefcn, t_span, reshape(X_0, [], 1)).', n, N_a, num_steps); % Column vector

% u_save = reshape(u_save, m, N_a, length(u_save));

X_d = zeros(n,N_a,num_steps);
t_index = 0;
for t = 0:t_step:t_end
    t_index = t_index + 1;
    X_d(:,:,t_index) = Functions.calculate_desired_trajectories(t);
end

%% Plot results
close all;
fontsize = 16;
% Functions.plot_real_time_trajectories(X, X_d, t_end, t_step, num_steps, fontsize);
% plot(X(:,1), X(:,2)); hold on;
global x_APF
x_APF = squeeze(X(:,1,:)).';
Figure_plot()
% Functions.plot_real_time_trajectories(X, num_steps);

