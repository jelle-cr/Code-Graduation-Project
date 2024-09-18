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
r_o = 0.4;                  % Radius of obstacle
p_d = rand(2, 1)*((range-1)+(range-1))-(range-1);	% Desired position

% Initial positions
p_0 = Functions.generate_initial_positions(N_a, r_a, range, p_o, r_o);

% Simulation time
t_end = 2.5;
t_step = 0.01;
t = 0:t_step:t_end;  % simulation time

% Save necessary parameters
% load('Data/Parameters.mat');
save('Data/Parameters.mat', 'A', 'B', 'n', 'm', 'N_a', 'r_a', 'u_max', 't_step', 'p_0', 'p_d', 'rho_0', 'K_att', 'K_rep', 'N_o', 'p_o', 'r_o');

fprintf('Saved System Parameters\n');

%% Calculate Potential Field
U_att = zeros(length(x), length(y));
F_att = zeros(length(x), length(y), height(p_d));
U_rep = zeros(length(x), length(y));
F_rep = zeros(length(x), length(y), height(p_d));
for i = 1:length(x)
    for j = 1:length(y)
        p = [x(i); y(j)];
        U_att(i,j) = 1/2*K_att*norm(p - p_d);
        F_att(i,j,:) = K_att*(p - p_d);

        for o = 1:N_o
            rho = norm(p - p_o(:,o)) - r_a - r_o;   
            if rho < rho_0
                rho = max(rho, 1e-6);           % In order to fill in the cylinder
                U_rep(i,j) = U_rep(i,j) + 1/2*K_rep*(1/rho - 1/rho_0)^2;
                if rho > 1e-1                   % Only calculate this outside of the obstacles, otherwise we get scaling issues
                    F_rep(i,j,:) = squeeze(F_rep(i,j,:)) -K_rep/rho^2*(1/rho - 1/rho_0)*(p - p_o(:,o))/norm(p - p_o(:,o));
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
[p] = reshape(Functions.ode4(@Functions.odefcn, t, reshape(p_0, [], 1)).', n, N_a, length(t)); % Column vector
fprintf('Simulation Done\n');

%% Plot results
close all
t_stop = t_end;
Functions.plot_real_time_trajectories(x, y, Potential, Force, range, localMinX, localMinY, globalMinX, globalMinY, p, t, t_stop);