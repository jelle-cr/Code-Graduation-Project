close all
clear all
clc

global u_att_save u_rep_save

% Quadcopter parameters
dimensions = 2;          % Number of axis (x,y,z)
states = 2*dimensions;   % Number of states
N_a = 2;                 % Number of agents
m = 0.01;                % Mass [kg]
d = 0.1;                 % Damping coefficient [Ns/m]
r_a = 0.05;              % Radius of agent [m]
u_max = 1;               % Maximum control force [N]
a_max = 1/m*norm([u_max; u_max]);   % Maximum acceleration [m/s^2]

% Initial positions
agent_spacing = 0.5;     % Spacing of formation circle agents (0.3 or 0.5 works well)
p0 = Functions.generate_circular_initial_positions(N_a, r_a, agent_spacing);
if states == 2*dimensions   % Add initial velocities to the states
    p0 = [p0; zeros(dimensions, N_a)];
end

% Nominal trajectories
use_V_ref = true;       % Determines whether or not to use reference velocity in CLF nominal control calculation
origin_max = 0.1;
origin_min = -origin_max;
A_min = 0.1;
A_max = 0.2;
f_min = 1;
f_max = 2;
phi_max = pi;
phi_min = -pi;
origin_rand = (origin_max-origin_min)*rand(2,N_a)+origin_min;
A_rand = (A_max-A_min)*rand(1,N_a)+A_min;
f_rand = (f_max-f_min)*rand(1,N_a)+f_min;
phi_rand = (phi_max-phi_min)*rand(1,N_a)+phi_min;
sign_rand = sign(randi([0, 1], dimensions, N_a) - 0.5);

% load('Data/FixedTrajectoryParameters.mat');    % Uncomment to use specific saved nominal trajectories
save('Data/TrajectoryParameters.mat', 'origin_rand', 'A_rand', 'f_rand', 'phi_rand', 'sign_rand', 'use_V_ref', 'N_a');
% % save('Data/FixedTrajectoryParameters.mat', 'origin_rand', 'A_rand', 'f_rand', 'phi_rand', 'sign_rand', 'use_V_ref', 'N_a');

% APF parameters
K_att_p = 0.5;
K_att_v = 0.25;
K_rep = 0.00001;
rho_0 = 2*r_a;

save('Data/Parameters.mat', 'dimensions', 'states', 'N_a', 'm', 'd', 'r_a', 'u_max', 'a_max', 'p0', 'K_att_p', 'K_att_v', 'K_rep', 'rho_0');

% Time vector
t_end = 5;
t_step = 0.01;
t_span = 0:t_step:t_end;  % simulation time
num_steps = length(t_span);

[p] = reshape(Functions.ode4(@Functions.odefcn, t_span, reshape(p0, [], 1)).', height(p0), N_a, num_steps);   % p is of size 4 by N_a by t

maxVelocity = max(max(max(p(3:4,:,:))))

u_att_save = reshape(u_att_save, dimensions, N_a, length(u_att_save));
u_rep_save = reshape(u_rep_save, dimensions, N_a, length(u_rep_save));

p_nom = zeros(states,N_a,num_steps);
u_att = zeros(dimensions,N_a,num_steps);
u_rep = zeros(dimensions,N_a,num_steps);
for t_index = 1:num_steps
    t = (t_index-1)*t_step;
    p_nom(:,:,t_index) = Functions.calculate_nominal_trajectories(t);
    u_att(:,:,t_index) = u_att_save(:,:,1+(t_index-1)*4);
    u_rep(:,:,t_index) = u_rep_save(:,:,1+(t_index-1)*4);
end

%% Average position error
err = 0;
for t = 1:num_steps
    for i = 1:N_a
        diff = squeeze(p_nom(1:2,i,t))-squeeze(p(1:2,i,t)); 
        err = err + norm(diff);
    end
end
avg_pos_err = err/(num_steps*N_a)

%% Plot results
close all;
update_interval = 1;     % How many time steps to skip before updating the plot
axlimit = max(abs(min(min(min(p(1:2,:,:))))), max(max(max(p(1:2,:,:)))))+r_a;  % Find abs max position value, add r_a to always be within frame        
xlimits = 1.2*[-axlimit axlimit];
ylimits = xlimits; 
fontsize = 18;
markersize = 10;
linewidth = 2;
t_stop = t_span(end);    % Determines when to freeze the updating plot
pauseplotting = false;   % Pauses the plot to set up recording software

Functions.plot_real_time_trajectories(p(1:states,:,:), t_step, N_a, update_interval, xlimits, ylimits, fontsize, r_a, rho_0, linewidth, p_nom(1:2,:,:), u_att, u_rep, num_steps, t_span, t_stop, pauseplotting); 