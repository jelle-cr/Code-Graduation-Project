close all
clear all
clc

% Objective: Trajectory tracking and Collision Avoidance with CLF+CBF
global u_save u_nom_save

% Quadcopter parameters
dimensions = 2;          % Number of axis (x,y,z)
states = 2*dimensions;   % Number of states
N_a = 4;                 % Number of agents
m = 0.01;                % Mass
d = 0.1;                 % Damping coefficient
agent_radius = 0.05;     % Radius of agent
u_max = 10;              % Maximum control force

% Initial positions
agent_spacing = 0.3;     % Spacing of formation circle agents (0.3 or 0.5 works well)
p0 = generate_circular_initial_positions(N_a, agent_radius, agent_spacing);
if states == 2*dimensions   % Add initial velocities to the states
    p0 = [p0; zeros(dimensions, N_a)];
end

% CBF parameters for safety filter
l0 = 600;           
l1 = 50;
D = l1^2-4*l0   
roots = -l1 + sqrt(D)   % Check if roots are negative
pause(0.5)
% Full responsibility mu = 1, or half responsibility mu = 1/2
mu = 1/2;

% CLF parameters for nominal control
l2 = 20;
l3 = 20;
lambda = 50;

% Add possible static obstacles
% ob = [-2; -2.5];
% r_ob = 1.5;
save('Parameters.mat', 'dimensions', 'states', 'N_a', 'm', 'd', 'agent_radius', 'u_max', 'p0', 'l0', 'l1', 'mu', 'l2', 'l3', 'lambda');

% Nominal trajectories
use_V_ref = false;      % Determines whether or not to use reference velocity in CLF nominal control calculation
origin_max = 0.2;
origin_min = -origin_max;
A_min = 0.2;
A_max = 0.5;
f_min = 1;
f_max = 3;
phi_max = pi;
phi_min = -pi;
origin_rand = (origin_max-origin_min)*rand(2,N_a)+origin_min;
A_rand = (A_max-A_min)*rand(1,N_a)+A_min;
f_rand = (f_max-f_min)*rand(1,N_a)+f_min;
phi_rand = (phi_max-phi_min)*rand(1,N_a)+phi_min;
sign_rand = sign(randi([0, 1], dimensions, N_a) - 0.5);
save('TrajectoryParameters.mat', 'origin_rand', 'A_rand', 'f_rand', 'phi_rand', 'sign_rand', 'use_V_ref');


% Time vector
t_end = 2;
t_step = 0.01;
t_span = 0:t_step:t_end;  % simulation time
num_steps = length(t_span);

[p] = reshape(ode4(@odefcn, t_span, reshape(p0, [], 1)).', height(p0), N_a, num_steps);   % p is of size 4 by N_a by t

u_nom_save = reshape(u_nom_save, dimensions, N_a, length(u_nom_save));
u_save = reshape(u_save, dimensions, N_a, length(u_save));

p_nom = zeros(states,N_a,num_steps);
u_nom = zeros(dimensions,N_a,num_steps);
u = zeros(dimensions,N_a,num_steps);
for t_index = 1:num_steps
    t = (t_index-1)*t_step;
    p_nom(:,:,t_index) = nominal_trajectories(t);
    u_nom(:,:,t_index) = u_nom_save(:,:,1+(t_index-1)*4);
    u(:,:,t_index) = u_save(:,:,1+(t_index-1)*4);
end

%% Plot results
close all;
update_interval = 0;
xlimits = 1.2*[-(A_max+origin_max) (A_max+origin_max)];
ylimits = 1.2*[-(A_max+origin_max) (A_max+origin_max)];
fontsize = 14;
markersize = 10;
linewidth = 2;
t_stop = t_span(end);    % Determines when to freeze the updating plot


plot_real_time_trajectories(p(1:states,:,:), t_step, N_a, update_interval, xlimits, ylimits, fontsize, agent_radius, linewidth, p_nom(1:2,:,:), u_nom, u, num_steps, t_span, t_stop); 



