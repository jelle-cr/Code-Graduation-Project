close all
clear all
clc
warning on; 

global u_att_save u_rep_save u_save

overrideNominalTrajectory = true;

% Quadcopter parameters
dimensions = 2;          % Number of axis (x,y,z)
states = 2*dimensions;   % Number of states
N_a = 2;                 % Number of agents
m = 0.01;                % Mass [kg]
d = 0.1;                 % Damping coefficient [Ns/m]
r_a = 0.05;              % Radius of agent [m]
u_max = 1;               % Maximum control force [N]
v_max = sqrt(2)/d*u_max; % Maximum velocity [m/s] (Absolute max in any direction)
a_max = 1/m*u_max;       % Maximum acceleration [m/s^2] (Single axial direction max)

% Initial positions
agent_spacing = 0.5;     % Spacing of formation circle agents (0.3 or 0.5 works well)
X_0 = Functions.generate_circular_initial_positions(N_a, r_a, agent_spacing);
if states == 2*dimensions   % Add initial velocities to the states
    X_0 = [X_0; zeros(dimensions, N_a)];
end
if overrideNominalTrajectory
    X_0 = [-0.4 0.4; 0 0; 0 0; 0 0];
end

% Nominal trajectories
use_V_ref = true;       % Determines whether or not to use reference velocity in CLF nominal control calculation
origin_max = 0.1;
origin_min = -origin_max;
A_min = 0.1;
A_max = 0.2;
f_min = 10;
f_max = 20;
phi_max = pi;
phi_min = -pi;
origin_rand = (origin_max-origin_min)*rand(2,N_a)+origin_min;
A_rand = (A_max-A_min)*rand(1,N_a)+A_min;
f_rand = (f_max-f_min)*rand(1,N_a)+f_min;
phi_rand = (phi_max-phi_min)*rand(1,N_a)+phi_min;
sign_rand = sign(randi([0, 1], dimensions, N_a) - 0.5);

% load('Data/FixedTrajectoryParameters.mat');    % Uncomment to use specific saved nominal trajectories
save('Data/TrajectoryParameters.mat', 'origin_rand', 'A_rand', 'f_rand', 'phi_rand', 'sign_rand', 'use_V_ref', 'N_a');
% save('Data/FixedTrajectoryParameters.mat', 'origin_rand', 'A_rand', 'f_rand', 'phi_rand', 'sign_rand', 'use_V_ref', 'N_a');

% APF parameters
K_att_p = 5;
K_att_v = 0.25;
K_rep = 0.00001;
rho_0 = 2*r_a;
% rho_0 = 2;
% if rho_0 < v_max^2/a_max
%     rho_0 = v_max^2/a_max;
%     warning('Automatically increased region of repulsion')
%     pause(1)
% end

save('Data/Parameters.mat', 'dimensions', 'states', 'N_a', 'm', 'd', 'r_a', 'u_max', 'a_max', 'X_0', 'K_att_p', 'K_att_v', 'K_rep', 'rho_0', 'overrideNominalTrajectory');

% Time vector
t_end = 0.2;
t_step = 0.001;
t_span = 0:t_step:t_end;  % simulation time
num_steps = length(t_span);

[X] = reshape(Functions.ode4(@Functions.odefcn, t_span, reshape(X_0, [], 1)).', height(X_0), N_a, num_steps);   % X is of size 4 by N_a by t

maxVelocity = max(max(max(X(3:4,:,:))))

u_att_save = reshape(u_att_save, dimensions, N_a, length(u_att_save));
u_rep_save = reshape(u_rep_save, dimensions, N_a, length(u_rep_save));
u_save = reshape(u_save, dimensions, N_a, length(u_save));

X_nom = zeros(states,N_a,num_steps);
u_att = zeros(dimensions,N_a,num_steps);
u_rep = zeros(dimensions,N_a,num_steps);
u = zeros(dimensions,N_a,num_steps);
for t_index = 1:num_steps
    t = (t_index-1)*t_step;
    X_nom(:,:,t_index) = Functions.calculate_nominal_trajectories(t,overrideNominalTrajectory,X_0);
    u_att(:,:,t_index) = u_att_save(:,:,1+(t_index-1)*4);
    u_rep(:,:,t_index) = u_rep_save(:,:,1+(t_index-1)*4);
    u(:,:,t_index) = u_save(:,:,1+(t_index-1)*4);
end

%%
u = zeros(dimensions,N_a,num_steps);
u_att = zeros(dimensions,N_a,num_steps);
u_rep = zeros(dimensions,N_a,num_steps);
u(:,:,1) = u_save(:,:,1);
u_att(:,:,1) = u_att_save(:,:,1);
u_rep(:,:,1) = u_rep_save(:,:,1);
for t_index = 2:num_steps
    temp1 = 0;
    temp2 = 0;
    temp3 = 0;
    for i = 1:4
        temp1 = temp1 + u_save(:,:,1+i+(t_index-2)*4);
        temp2 = temp2 + u_att_save(:,:,1+i+(t_index-2)*4);
        temp3 = temp3 + u_rep_save(:,:,1+i+(t_index-2)*4);
    end
    u(:,:,t_index) = 0.25*temp1;
    u_att(:,:,t_index) = 0.25*temp2;
    u_rep(:,:,t_index) = 0.25*temp3;
end

%% Average position error
err = 0;
for t = 1:num_steps
    for i = 1:N_a
        diff = squeeze(X_nom(1:2,i,t))-squeeze(X(1:2,i,t)); 
        err = err + norm(diff);
    end
end
avg_pos_err = err/(num_steps*N_a)
dist = abs(X(1,1,:) - X(1,2,:))-2*r_a;
minDist = min(min(min(dist)))

%% Plot results
close all;
update_interval = 1;     % How many time steps to skip before updating the plot
axlimit = max(abs(min(min(min(X(1:2,:,:))))), max(max(max(X(1:2,:,:)))))+r_a;  % Find abs max position value, add r_a to always be within frame        
xlimits = 1.2*[-axlimit axlimit];
ylimits = xlimits; 
fontsize = 18;
markersize = 10;
linewidth = 2;
t_stop = t_span(end);    % Determines when to freeze the updating plot
pauseplotting = false;   % Pauses the plot to set up recording software

Functions.plot_real_time_trajectories(X(1:states,:,:), t_step, N_a, update_interval, xlimits, ylimits, fontsize, r_a, rho_0, linewidth, X_nom(1:2,:,:), u_att, u_rep, u, num_steps, t_span, t_stop, pauseplotting); 