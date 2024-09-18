close all
clear all

range = 3;
num_steps = 500;
x = linspace(-range, range, num_steps);
y = linspace(-range, range, num_steps);

%% Simulation parameters
N_a = 1;                    % Number of trajectories to simulate
N_o = 1;                    % Number of obstacles
A = [0, 0;
     0, 0];
B = [1, 0;
     0, 1];
n = height(A);
m = width(B);
u_max = 10;

% Potential field parameters
K_att = 1;
K_rep = 0.001;
rho_0 = 0.5;
p_o = rand(2,N_o)*((range-1)+(range-1))-(range-1);

r_a = 0.5;                  % Radius of agent
r_o = 0.5;                  % Radius of obstacle
p_d = rand(2, 1)*((range-1)+(range-1))-(range-1);	% Desired position
p_d = [2; 2];

% Initial positions
p_0 = Functions.generate_initial_positions(N_a, r_a, range, p_o, r_o);
p_0 = [-2;-2];

% Simulation time
t_end = 3;
t_step = 0.01;
t = 0:t_step:t_end;  % simulation time

% Save necessary parameters
% load('Data/Parameters.mat');
save('Data/Parameters.mat', 'A', 'B', 'n', 'm', 'N_a', 'r_a', 'u_max', 't_step', 'p_0', 'p_d', 'rho_0', 'K_att', 'K_rep', 'N_o', 'p_o', 'r_o');

fprintf('Saved System Parameters\n');


%% Simulate
[p] = reshape(Functions.ode4(@Functions.odefcn, t, reshape(p_0, [], 1)).', n, N_a, length(t)); % Column vector
fprintf('Simulation Done\n');

%% Plot results
close all
delay = 0;
t_stop = t_end;
Functions.plot_real_time_trajectories(range, p, t, t_stop, delay);
                                    