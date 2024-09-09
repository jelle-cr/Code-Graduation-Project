% Double integrator system
% close all
clear all

range = 3;
num_steps = 500;
x = linspace(-range, range, num_steps);
y = linspace(-range, range, num_steps);

controller = 'APF';
controller = 'semi-APF';
controller = 'CLF-CBF';

%% Simulation parameters
N_a = 1;            % Number of trajectories to simulate
N_o = 3;            % Number of obstacles
A = [0, 0, 1, 0;    % State space
     0, 0, 0, 1;
     0, 0, 0, 0;
     0, 0, 0, 0];
B = [0, 0;
     0, 0;
     1, 0;
     0, 1];
n = height(A);      % Number of states
m = width(B);       % Number of inputs
u_max = 10;         % Maximum control input in 1 direction
r_a = 0.5;          % Radius of agent 

% Potential field parameters
K_att_p = 5;        % Attractive position gain
K_att_v = 1;        % Attractive velocity gain
K_rep = 0.001;      % Repulsive gain
rho_0 = 0.1;        % Repulsive potential range
r_o = 0.4;          % Radius of obstacle

% Obstacle states
q_o = [rand(2, N_o)*((range-1)+(range-1))-(range-1);
       zeros(2, N_o)];
q_o = [1.25, -1, -1;
       1, 1, -1.25;
       0, 0, 0;
       0, 0, 0];

% Desired state(s)
q_d = [rand(2, 1)*((range-1)+(range-1))-(range-1);	 
       0*ones(2, 1)];
q_d = [2;2;0;0];

% Initial states
q_0 = [Functions.generate_initial_positions(N_a, r_a, range, q_o(1:2,:), r_o);
       0*ones(2, N_a)];
q_0 = [-2.5;-2.5;0;0];

% Simulation time
t_end = 10;
t_step = 0.01;
t = 0:t_step:t_end;  % simulation time

% Save necessary parameters
% load('Data/Parameters.mat');
save('Data/Parameters.mat', 'controller', 'A', 'B', 'n', 'm', 'N_a', 'r_a', 'u_max', 't_step', 'q_0', 'q_d', 'rho_0', 'K_att_p', 'K_att_v', 'K_rep', 'N_o', 'q_o', 'r_o');

fprintf('Saved System Parameters\n');

%% Calculate Positional Potential Field
p_o = q_o(1:2,:);
p_d = q_d(1:2);

U_att = zeros(length(x), length(y));
F_att = zeros(length(x), length(y), height(p_d));
U_rep = zeros(length(x), length(y));
F_rep = zeros(length(x), length(y), height(p_d));
for i = 1:length(x)
    for j = 1:length(y)
        p = [x(i); y(j)];
        U_att(i,j) = 1/2*K_att_p*norm(p - p_d(1:2));
        F_att(i,j,:) = K_att_p*(p - p_d(1:2));

        for o = 1:N_o
            rho = norm(p - p_o(1:2,o)) - r_a - r_o;   
            if rho < rho_0
                rho = max(rho, 1e-6);           % In order to fill in the cylinder
                U_rep(i,j) = U_rep(i,j) + 1/2*K_rep*(1/rho - 1/rho_0)^2;
                if rho > 1e-1                   % Only calculate this outside of the obstacles, otherwise we get scaling issues
                    F_rep(i,j,:) = squeeze(F_rep(i,j,:)) -K_rep/rho^2*(1/rho - 1/rho_0)*(p - p_o(1:2,o))/norm(p - p_o(1:2,o));
                else
                    F_att(i,j,:) = [0; 0];      % We don't want any quivers to be drawn near the obstacle
                end
            end
        end
    end
end

Potential = U_att + U_rep;
Potential = min(Potential, 1*max(max(U_att)));
Force = -F_att - F_rep;

% Find global minimum
minPotential = min(min(Potential));
[globalMinX_idx, globalMinY_idx] = find(Potential == minPotential);
globalMinX = x(globalMinX_idx);
globalMinY = y(globalMinY_idx);

% Find local minima using imregionalmin
localMinima = imregionalmin(Potential);     % Way faster than looping through matrix manually
[localMinX_idx, localMinY_idx] = find(localMinima);
localMinX = x(localMinX_idx);
localMinY = y(localMinY_idx);

fprintf('Potential Field Generated\n');

%% Simulate
[q] = reshape(Functions.ode4(@Functions.odefcn, t, reshape(q_0, [], 1)).', n, N_a, length(t)); % Column vector
p = q(1:2,:,:);
fprintf('Simulation Done\n');

%% Plot results
delay = 0;

close all
t_stop = t_end;
Functions.plot_real_time_trajectories(x, y, Potential, range, localMinX, localMinY, globalMinX, globalMinY, p, t, t_stop, delay);