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
p0 = Functions.generate_circular_initial_positions(N_a, agent_radius, agent_spacing);
if states == 2*dimensions   % Add initial velocities to the states
    p0 = [p0; zeros(dimensions, N_a)];
end

% Nominal trajectories
use_V_ref = false;       % Determines whether or not to use reference velocity in CLF nominal control calculation
origin_max = 0.1;
origin_min = -origin_max;
A_min = 0.1;
A_max = 0.3;
f_min = 2;
f_max = 3;
phi_max = pi;
phi_min = -pi;
origin_rand = (origin_max-origin_min)*rand(2,N_a)+origin_min;
A_rand = (A_max-A_min)*rand(1,N_a)+A_min;
f_rand = (f_max-f_min)*rand(1,N_a)+f_min;
phi_rand = (phi_max-phi_min)*rand(1,N_a)+phi_min;
sign_rand = sign(randi([0, 1], dimensions, N_a) - 0.5);

% load('Data/FixedTrajectoryParameters.mat');    % Uncomment to use specific saved nominal trajectories
save('Data/TrajectoryParameters.mat', 'origin_rand', 'A_rand', 'f_rand', 'phi_rand', 'sign_rand', 'use_V_ref', 'N_a');

% CBF parameters for safety filter
l0 = 600;           
l1 = 50;
D = l1^2-4*l0   
roots = -l1 + sqrt(D)   % Check if roots are negative
pause(0.5)
% Calculate mu based on the assigned agent weights, higher weight allows agent to stay closer to reference position
agent_responsibility_weights = ones(N_a,1); % Value between 0 and 1, the ratio for each agent determines the value of mu
mu = Functions.calculate_agent_mu(N_a, agent_responsibility_weights);

% CLF parameters for nominal control
l2 = 20;
l3 = 20;
lambda = 50;

save('Data/Parameters.mat', 'dimensions', 'states', 'N_a', 'm', 'd', 'agent_radius', 'u_max', 'p0', 'l0', 'l1', 'mu', 'l2', 'l3', 'lambda');

% Time vector
t_end = 2;
t_step = 0.005;
t_span = 0:t_step:t_end;  % simulation time
num_steps = length(t_span);

[p] = reshape(Functions.ode4(@Functions.odefcn, t_span, reshape(p0, [], 1)).', height(p0), N_a, num_steps);   % p is of size 4 by N_a by t

u_nom_save = reshape(u_nom_save, dimensions, N_a, length(u_nom_save));
u_save = reshape(u_save, dimensions, N_a, length(u_save));

p_nom = zeros(states,N_a,num_steps);
u_nom = zeros(dimensions,N_a,num_steps);
u = zeros(dimensions,N_a,num_steps);
for t_index = 1:num_steps
    t = (t_index-1)*t_step;
    p_nom(:,:,t_index) = Functions.calculate_nominal_trajectories(t);
    u_nom(:,:,t_index) = u_nom_save(:,:,1+(t_index-1)*4);
    u(:,:,t_index) = u_save(:,:,1+(t_index-1)*4);
end

%% Plot results
close all;
update_interval = 0;
axlimit = max(abs(min(min(min(p(1:2,:,:))))), max(max(max(p(1:2,:,:)))))+agent_radius;  % Find abs max position value, add agent_radius to always be within frame        
xlimits = 1.2*[-axlimit axlimit];
ylimits = xlimits; 
fontsize = 14;
markersize = 10;
linewidth = 2;
t_stop = t_span(end);    % Determines when to freeze the updating plot

Functions.plot_real_time_trajectories(p(1:states,:,:), t_step, N_a, update_interval, xlimits, ylimits, fontsize, agent_radius, linewidth, p_nom(1:2,:,:), u_nom, u, num_steps, t_span, t_stop); 

