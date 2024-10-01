% Double integrator system
% close all
clear all
clc

range = 3;
num_steps = 500;
x = linspace(-range, range, num_steps);
y = linspace(-range, range, num_steps);

controller = 'APF';

%% Simulation parameters
N_a = 1;            % Number of trajectories to simulate
N_o = 1;            % Number of obstacles
A = [0, 0, 1, 0;    % State space
     0, 0, 0, 1;
     0, 0, 0, 0;
     0, 0, 0, 0];
B = [0, 0;
     0, 0;
     1, 0;
     0, 1];
A = [0, 0;    % State space
     0, 0];
B = [1, 0;
     0, 1];
n = height(A);      % Number of states
m = width(B);       % Number of inputs
u_max = 30;         % Maximum control input in 1 direction
r_a = 0.5;          % Radius of agent 

% Potential field parameters
k_att_p = 1;        % Attractive position gain
k_att_v = 1;        % Attractive velocity gain
k_att_c = 1;        % Attractive coupling gain, note k_p*k_v > k_c^2
if k_att_p*k_att_v <= k_att_c^2
    warning('W matrix is not positive definite');
end
W_att = [k_att_p*eye(n/2), k_att_c*eye(n/2); 
         k_att_c*eye(n/2), k_att_v*eye(n/2)];
W_att = k_att_p*eye(n);
k_rep = 0.001;      % Repulsive gain
rho_0 = 0.2;        % Repulsive potential range
r_o = 0.6;          % Radius of obstacle

% Obstacle states
q_o = [rand(2, N_o)*((range-1)+(range-1))-(range-1);
       zeros(2, N_o)];
% q_o = [-1, -1, 1.25;
%        -1.25, 1, 1;
%        0, 0, 0;
%        0, 0, 0];
q_o = [-0.1;
       -0.4;
       0;
       0];
q_o = [-0.1;
       -0.4];

% Desired state(s)
q_d = [rand(2, 1)*((range-1)+(range-1))-(range-1);	 
       0*ones(2, 1)];
q_d = [2;1;0;0];
q_d = [2;1];

% Initial states
q_0 = [Functions.generate_initial_positions(N_a, r_a, range, q_o(1:2,:), r_o);
       0*ones(2, N_a)];
q_0 = [-2.5;-2.5;0;0];
q_0 = [-2.5;-2.5];
% Simulation time
t_end = 5;
t_step = 0.01;
t = 0:t_step:t_end;  % simulation time

% Save necessary parameters
% load('Data/Parameters.mat');
save('Data/Parameters.mat', 'controller', 'A', 'B', 'n', 'm', 'N_a', 'r_a', 'u_max', 't_step', 'q_0', 'q_d', 'rho_0', 'W_att', 'k_rep', 'N_o', 'q_o', 'r_o');

fprintf('Saved System Parameters\n');

%% Simulate
tic
[q] = reshape(Functions.ode4(@Functions.odefcn, t, reshape(q_0, [], 1)).', n, length(t)); % Column vector
% [t, q] = ode45(@Functions.odefcn, [0 t(end)], reshape(q_0, [], 1));
% q = reshape(q.',n,N_a,length(t));
p = q(1:2,:);
fprintf('Simulation Done\n');
toc

%% Plot results
close all
delay = 0;

t_stop = t_end;
Functions.plot_real_time_trajectories(p, range, t_stop, t_step, delay);

