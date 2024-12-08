% Double integrator system
close all
clear all

dynamics = 'Double Integrator';
environment = 'tripleObstacle'; N_o = 3;
% environment = 'corridor'; N_o = 2;
% environment = 'goalNearObstacle'; N_o = 1;
controller = 'APF'; 
controller = 'APF-SafetyFilter';
% controller = 'CBF';

%% Simulation parameters
N_a = 1;            % Number of trajectories to simulate
% N_o = 3;            % Number of obstacles

M = 1;              % Mass
d = 0;              % Damping coefficient

A = [0, 0, 1, 0;    % State space
     0, 0, 0, 1;
     0, 0, -d/M, 0;
     0, 0, 0, -d/M];
B = [0, 0;
     0, 0;
     1/M, 0;
     0, 1/M];

n = height(A);      % Number of states
m = width(B);       % Number of inputs
u_max = 30;         % Maximum control input in 1 direction
r_a = 0;            % Radius of agent 
r_o = 0.75;         % Radius of obstacle

% Potential field parameters
k_att = 1;          % Attractive potential gain
k_rep = 0.01;       % Repulsive potential gain
rho_0 = 3;        % Repulsive potential range

[p_0, p_d, p_o] = Functions.environment_setup(environment, dynamics, N_a);
x_0 = [p_0; 0*p_0];
x_d = [p_d; 0*p_d];
x_o = [p_o; 0*p_o];

% Simulation time
t_end = 5;
t_step = 0.01;
t = 0:t_step:t_end;  % simulation time
num_steps = length(t);

% Save necessary parameters
save('Data/Parameters.mat', 'N_a', 'N_o', 'A', 'B', 'n', 'm', 'u_max', 'r_a', 'r_o', ...
                            'k_att', 'k_rep', 'rho_0', 'x_0', 'x_d', 'x_o', ...
                            't_end', 't_step', 'dynamics', 'controller');

fprintf('Saved System Parameters\n');

%% Simulate
tic
x = reshape(Functions.ode4(@Functions.odefcn, t, reshape(x_0, [], 1)).', n, N_a, length(t)); % Column vector
fprintf('Simulation Done\n');
toc
x = x(1:2,:,:); %Extract position

% Save complete state and control input 
load('Data/SimulationDataRecent.mat');    % Loads u that was saved
save('Data/SimulationDataRecent.mat', 'x', 'u_att', 'u_rep', 'N_a', 'N_o', 'A', 'B', 'n', 'm', 'u_max', 'r_a', 'r_o', ...
                                           'k_att', 'k_rep', 'rho_0', 'x_0', 'x_d', 'x_o', ...
                                           't_end', 't_step', 'controller');
delete('Data/Parameters.mat');

%% Plot potentials
save = false;
% Control input
u_norm = zeros(N_a,length(t));
u_att_norm = zeros(N_a,length(t));
u_rep_norm = zeros(N_a,length(t));
for t_ind = 1:length(t)
    for i = 1:N_a
        u_norm(i,t_ind) = 1/2*norm(squeeze(u_att(:,i,t_ind)) + squeeze(u_rep(:,i,t_ind)))^2;
        u_att_norm(i,t_ind) = 1/2*norm(squeeze(u_att(:,i,t_ind)))^2;
        u_rep_norm(i,t_ind) = 1/2*norm(squeeze(u_rep(:,i,t_ind)))^2;
    end
end
Functions.plot_over_time(u_norm, t_step, t_end, '\frac{1}{2}||\mathbf{u}||^2', save);
% Functions.plot_over_time(u_rep_norm, t_step, t_end, '\frac{1}{2}||\mathbf{u}_{rep}||^2', save);
u_norm_avg = mean(u_norm,2)
u_rep_norm_avg = mean(u_rep_norm,2)

% Attractive potential
U_att = zeros(N_a, length(t));
for t_ind = 1:length(t)
    for i = 1:N_a
        U_att(i,t_ind) = 1/2*norm(x(1:2,i,t_ind)-x_d(1:2))^2;
    end
end
% Functions.plot_over_time(U_att, t_step, t_end, 'U_{att}(\mathbf{x})', save);

% Repulsive potential
U_rep = zeros(N_o, length(t));
h = zeros(N_o, length(t));
for t_ind = 1:length(t)
    for j = 1:N_o
        p_ij = x(1:2,1,t_ind)-x_o(1:2,j);
        h(j, t_ind) = norm(p_ij) - r_a - r_o;
        if h(j, t_ind) < rho_0
            U_rep(j,t_ind) = U_rep(j,t_ind) + 1/2*k_rep*(1/h(j, t_ind)-1/rho_0)^2;
        end
    end
end
% Functions.plot_over_time(h, t_step, t_end, 'h(\mathbf{x})', save);
% Functions.plot_over_time(U_rep, t_step, t_end, 'U_{rep}(\mathbf{x})', save);

%% Plot trajectories
rangeX = [-3; 3];
rangeY = [-2; 2];
plottingFolder = 'Data';
Functions.plot_static_trajectories(rangeX, rangeY, plottingFolder);

