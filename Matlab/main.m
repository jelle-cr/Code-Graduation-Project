%% Single integrator system
close all
clear all

dynamics = 'Single Integrator';
% dynamics = 'Double Integrator';
controller = 'APF'; 
controller = 'APF-SF';

%% Simulation parameters
% Setup parameters
N_a = 1;            % Number of agents to simulate
N_o = 3;            % Number of obstacles

% State space (to use control-affine dynamics, directly specify model in odefcn.m)
if strcmp(dynamics, 'Single Integrator')
    A = [0, 0;          
         0, 0];
    B = [1, 0;
         0, 1];
else
    A = [0, 0, 1, 0;    
         0, 0, 0, 1;
         0, 0, 0, 0;
         0, 0, 0, 0];
    B = [0, 0;
         0, 0;
         1, 0;
         0, 1];
end

n = height(A);      % Number of states
m = width(B);       % Number of inputs
r_a = 0;            % Radius of agent 
r_o = 0.5;          % Radius of obstacle

x_0 = [-4;          % Initial position
       -2];
x_d = [4;           % Desired position
       2];
x_o = [-2,   -0.5,  2;     % Obstacle positions
       -1.25, 1.25, 0.75];
if strcmp(dynamics, 'Double Integrator') % Append positions with velocities
    x_0 = [x_0; zeros(size(x_0))];
    x_d = [x_d; zeros(size(x_d))];
    x_o = [x_o; zeros(size(x_o))];
end

% Potential field parameters
k_att = 1;          % Attractive potential gain
k_rep = 1;          % Repulsive potential gain
h_0 = 1;            % Repulsive potential range

% Safety filter parameters
k_sigma = 1;        % sigma = k_sigma*norm(F_att)^2
k_gamma = 1;        % Gamma = k_gamma*norm(F_rep)^2
k_alpha = 1;        % alpha = k_alpha*min(h)

% Double integrator specific proportional feedback gain
k_pid = 1;

% Simulation time
t_end = 10;         % Simulate for 10 seconds
t_step = 0.001;     % Controller running at 1000 Hz
t = 0:t_step:t_end; % simulation time
num_steps = length(t);

% Save necessary parameters
save('Data/Parameters.mat', 'N_a', 'N_o', 'A', 'B', 'n', 'm', 'r_a', 'r_o', ...
                            'k_att', 'k_rep', 'h_0', 'x_0', 'x_d', 'x_o', ...
                            'k_sigma', 'k_gamma', 'k_alpha', 'k_pid', ...
                            't_end', 't_step', 'dynamics', 'controller');

fprintf('Saved System Parameters\n');

%% Simulate
tic
x = reshape(Functions.ode4(@Functions.odefcn, t, reshape(x_0, [], 1)).', n, N_a, length(t)); % Column vector
fprintf('Simulation Done\n');
toc

load('Data/SimulationDataRecent.mat');    % Loads u that was saved

%% Calculate control effort
u_norm = zeros(N_a,length(t));
for t_ind = 1:length(t)
    for i = 1:N_a
        u_norm(i,t_ind) = 1/2*norm(squeeze(u_att(:,i,t_ind)) + squeeze(u_rep(:,i,t_ind)))^2;
    end
end
u_norm_avg = mean(u_norm,2)

%% Save complete state and control input 
save('Data/SimulationDataRecent.mat', 'x', 'u_att', 'u_rep', 'N_a', 'N_o', 'A', 'B', 'n', 'm', 'r_a', 'r_o', ...
                                           'k_att', 'k_rep', 'h_0', 'x_0', 'x_d', 'x_o', ...
                                           'k_sigma', 'k_gamma', 'k_gamma', 'k_pid', 'u_norm_avg', ...
                                           't_end', 't_step', 'controller');
delete('Data/Parameters.mat');

%% Plot trajectories
rangeX = [-5; 5];
rangeY = [-3; 3];
plottingFolder = 'Data';
Functions.plot_static_trajectories(rangeX, rangeY, plottingFolder);

