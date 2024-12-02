close all
clear all

p_a = [2.5;2];
p_d = [4;4];
p_o = [2;3];
k_att = 1;
F_att = -k_att*(p_a-p_d);
k_rep = 1;
h = norm(p_a-p_o);
F_rep = k_rep/h^2*(p_a-p_o)/norm(p_a-p_o); % U_rep=1/h

u_APF = F_att + F_rep;

% APF safety filter
K_att = 1;
sigma = 0;
alpha = h;
K_rep = (sigma - alpha - K_att*F_rep.'*F_att)/norm(F_rep)^2
u_APFSF = K_att*F_att;
if K_rep > 0
    u_APFSF = u_APFSF + K_rep*F_rep;
end

rangeX = [0, 5]; rangeY = [0, 5];
colors = lines(5);

figure('Position', [700 150 750 750]);
hold on; grid on; axis equal;
xlim(rangeX); ylim(rangeY);
xticks(rangeX(1):1:rangeX(2)); yticks(rangeY(1):1:rangeY(2));
ax = gca; set(ax, 'FontSize', 16); ax.TickLabelInterpreter = 'latex';
xlabel('$x_1$', 'Interpreter','latex', 'FontSize', 25);
ylabel('$x_2$', 'Interpreter','latex', 'FontSize', 25);
plot(p_a(1), p_a(2), 'Marker','.', 'LineWidth', 3, 'MarkerSize', 35, 'Color', colors(1,:));
plot(p_o(1), p_o(2), 'Marker','+', 'LineWidth', 3, 'MarkerSize', 10, 'Color', colors(2,:)); 
plot(p_d(1), p_d(2), 'Marker','x', 'LineWidth', 3, 'MarkerSize', 10, 'Color', colors(5,:)); 
quiver(p_a(1), p_a(2), F_att(1), F_att(2), 'off', 'LineWidth', 3, 'Color', colors(5,:));
quiver(p_a(1), p_a(2), F_rep(1), F_rep(2), 'off', 'LineWidth', 3, 'Color', colors(2,:));
quiver(p_a(1), p_a(2), u_APF(1), u_APF(2), 'off', 'LineWidth', 3, 'Color', colors(3,:));
quiver(p_a(1), p_a(2), u_APFSF(1), u_APFSF(2), 'off', 'LineWidth', 3, 'Color', colors(4,:));

%% Plot of potential field
% close all
% clear all
% 
% range = 3;
% num_steps = 2500;
% x = linspace(-range, range, num_steps);
% y = linspace(-range, range, num_steps);
% 
% %% Simulation parameters
% N_a = 1;                    % Number of trajectories to simulate
% N_o = 1;                    % Number of obstacles
% A = [0, 0;
%      0, 0];
% B = [1, 0;
%      0, 1];
% n = height(A);
% m = width(B);
% u_max = 10;
% 
% % Potential field parameters
% K_att = 1;
% K_rep = 0.1;
% rho_0 = 1.5;
% p_o = [0;
%        0];
% 
% r_a = 0;                  % Radius of agent
% r_o = 0.5;                  % Radius of obstacle
% p_d = rand(2, 1)*((range-1)+(range-1))-(range-1);	% Desired position
% p_d = [-2;-2];
% 
% %% Calculate Potential Field
% U_att = zeros(length(x), length(y));
% U_rep = zeros(length(x), length(y));
% for i = 1:length(x)
%     for j = 1:length(y)
%         p = [x(i); y(j)];
%         U_att(i,j) = 1/2*K_att*norm(p - p_d);
% 
%         for o = 1:N_o
%             rho = norm(p - p_o(:,o)) - r_a - r_o;   
%             if rho < rho_0
%                 rho = max(rho, 1e-6);           % In order to fill in the cylinder
%                 U_rep(i,j) = U_rep(i,j) + 1/2*K_rep*(1/rho - 1/rho_0)^2;
%             end
%         end
%     end
% end
% 
% Potential = U_att + U_rep;
% Potential = min(Potential, 1*max(max(U_att)));
% % Potential = U_att;
% Potential =  min(U_rep, 1*max(max(U_att)));
% 
% % Normalize the potential field to the range [0, 1]
% Potential_min = min(Potential(:));  % Find the minimum value of the potential
% Potential_max = max(Potential(:));  % Find the maximum value of the potential
% Potential = (Potential - Potential_min) / (Potential_max - Potential_min);  % Remap to [0, 1]
% 
% fprintf('Potential Field Generated\n');
% 
% %% Plot results
% close all
% figure('Position', [100 50 800 700]);  %Left Bottom Width Height
% surf(x,y,Potential','FaceAlpha',1, 'EdgeColor','none')
% % plot(x,y)
% grid on;
% clim([0 1]);  % Ensure color scale goes from 0 to 1
% cb = colorbar;
% cb.Ticks = linspace(0, 1, 5); % Set colorbar ticks evenly
% cb.TickLabels = linspace(0, 1, 5); % Override labels from 0 to 1
% colormap('parula');
% ax = gca;
% ax.ZTick = linspace(0, 1, 5); % Set z-axis ticks evenly
% ax.ZTickLabel = linspace(0, 1, 5); % Override z-axis labels to match [0, 1]
% set(ax, 'FontSize', 12);
% xlim([-3 3]); ylim([-3 3]); zlim([0 1]);
% xlabel('$x_1$', 'Interpreter','latex', 'FontSize', 22);
% ylabel('$x_2$', 'Interpreter','latex', 'FontSize', 22);
% zlabel('$U_{tot}(\mathbf{x})$', 'Interpreter','latex', 'FontSize', 20);
% title('Repulsive Potential Function', 'Interpreter', 'latex', 'FontSize', 22);
% view(-15, 45);
% view(0, 90);

%% Plot of Lyapunov function
% close all
% clear all
% 
% 
% % Define the state space grid
% x1 = linspace(-2, 2, 100); % Range for x1
% x2 = linspace(-2, 2, 100); % Range for x2
% [X1, X2] = meshgrid(x1, x2);
% 
% % Define the Lyapunov function V(x)
% V = X1.^2 + X2.^2; % Corrected quadratic CLF
% 
% % Plot the 3D surface of V(x)
% figure;
% surf(X1, X2, V, 'EdgeColor', 'none', 'FaceAlpha', 0.7); % Semi-transparent surface
% hold on;
% colormap('parula'); % Use a default colormap
% colorbar;
% xlabel('$x_1$', 'Interpreter', 'latex', 'FontSize', 14);
% ylabel('$x_2$', 'Interpreter', 'latex', 'FontSize', 14);
% zlabel('$V(x)$', 'Interpreter', 'latex', 'FontSize', 14);
% title('3D Visualization of $V(x)$ and System Trajectories', 'Interpreter', 'latex', 'FontSize', 16);
% grid on;
% 
% % Define the system dynamics
% dynamics = @(t, x) [x(2); -x(1) - 2*x(2)]; % Example system
% 
% % Initial conditions for trajectories (on a circle of radius 1.75)
% radius = 1.75;
% num_points = 20; % Number of initial conditions
% angles = linspace(0, 2*pi, num_points); % Angles from 0 to 2*pi
% initial_conditions = [radius * cos(angles)', radius * sin(angles)'];
% 
% % Simulate and plot trajectories
% for i = 1:size(initial_conditions, 1)
%     x0 = initial_conditions(i, :);
% 
%     % Simulate the system
%     [T, Y] = ode45(dynamics, [0, 10], x0);
% 
%     % Compute V(x) along the trajectory
%     V_traj = Y(:, 1).^2 + Y(:, 2).^2;
% 
%     % Plot the trajectory on the 3D surface
%     plot3(Y(:, 1), Y(:, 2), V_traj, 'LineWidth', 1.5); % Trajectory
%     plot3(x0(1), x0(2), x0(1)^2 + x0(2)^2, 'o', 'MarkerSize', 8, 'LineWidth', 1.5); % Initial point
% end
% 
% % Mark the equilibrium point
% plot3(0, 0, 0, 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Equilibrium point at origin
% 
% % Finalize the plot
% view(45, 30); % Adjust the view angle for better visualization
% axis tight; % Ensure the axes fit tightly around the data
% hold off;
