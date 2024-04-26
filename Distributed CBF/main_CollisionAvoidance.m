close all
clear all

% Objective: Trajectory tracking and Collision Avoidance with CLF+CBF
global u_save u_nom

% Load system parameters
dimensions = 2;          % Number of axis (x,y,z)
states = 2*dimensions;   % Number of states
N_a = 6;                 % Number of agents
m = 0.01;                % Mass
d = 0.1;                 % Damping coefficient
agent_radius = 1;        % Radius of agent
agent_spacing = 0.3;     % Spacing of formation circle agents (0.5 works well)

p0 = generate_circular_initial_positions(N_a, agent_radius, agent_spacing);
if states == 2*dimensions   % Add initial velocities to the states
    p0 = [p0; zeros(dimensions, N_a)];
end
% Append size for nominal trajectory
p0 = [p0; zeros(size(p0))];

% Nominal control input
A_min = 8;
A_max = 12;
f_min = 8;
f_max = 12;
A_rand = (A_max-A_min)*rand(1,N_a)+A_min;
f_rand = (f_max-f_min)*rand(1,N_a)+f_min;
sign_rand = sign(randi([0, 1], dimensions, N_a) - 0.5);
nominal_tolerance = 0.01;

l0 = 500;           % Weights for CBF, Double integrator model
l1 = 50;
D = l1^2-4*l0
roots = -l1 + sqrt(D)
pause(0.5)

gamma_cbf = 30;     % Single integrator model

% ob = [-2; -2.5];
% r_ob = 1.5;

save('parameters.mat', 'dimensions', 'states', 'N_a', 'm', 'd', 'agent_radius', 'p0', 'A_rand', 'f_rand', 'sign_rand', 'nominal_tolerance', 'l0', 'l1', 'gamma_cbf');


% Time vector
t_step = 0.01;
t_span = 0:t_step:1;  % simulation time
num_steps = length(t_span);

[p] = reshape(ode4(@odefcn, t_span, reshape(p0, [], 1)).', height(p0), N_a, num_steps);   % p is of size 4 by N_a by t

u_nom = reshape(u_nom, dimensions, N_a, length(u_nom));
u_save = reshape(u_save, dimensions, N_a, length(u_save));
u0 = zeros(dimensions,N_a,num_steps);
u = zeros(dimensions,N_a,num_steps);
for t = 1:num_steps
    u0(:,:,t) = u_nom(:,:,1+(t-1)*4);
    u(:,:,t) = u_save(:,:,1+(t-1)*4);
end

%% Plot results
close all;
update_interval = 0;
xlimits = [min(min(p(1,:,:)))-2 max(max(p(1,:,:)))+2];
ylimits = [min(min(p(2,:,:)))-2 max(max(p(2,:,:)))+2];
fontsize = 14;
markersize = 10;
linewidth = 2;
t_stop = t_span(end);    % Determines when to freeze the updating plot


plot_real_time_trajectories(p, t_step, N_a, update_interval, xlimits, ylimits, fontsize, agent_radius, linewidth, u0, u, num_steps, t_span, t_stop); 


