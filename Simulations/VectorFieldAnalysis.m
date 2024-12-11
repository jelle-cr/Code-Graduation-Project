
%% Plot of potential field
close all
clear all

dynamics = 'Single Integrator';
% environment = 'singletary'; N_o = 2;
% environment = 'tripleObstacle'; N_o = 3;
% environment = 'corridor'; N_o = 2;
% environment = 'goalNearObstacle'; N_o = 1;

%% Cartesian coordinates
rangeX = [-1; 1];
rangeY = [-1; 1];
num_steps = 30;
x = linspace(rangeX(1), rangeX(2), num_steps);
y = linspace(rangeY(1), rangeY(2), num_steps);

%% Simulation parameters
N_a = 1;                    % Number of trajectories to simulate
N_o = 1;
A = [0, 0;
     0, 0];
B = [1, 0;
     0, 1];
n = height(A);
m = width(B);
u_max = 10;

% Potential field parameters
k_alpha = 1;
k_gamma = 0;
gamma_static = 10000000000;

k_att = 1;
k_rep = 0.1;
rho_0 = 1;
p_o = [0;
       0];

r_a = 0;                  % Radius of agent
r_o = 0.4;
% p_d = rand(2, 1)*((range-1)+(range-1))-(range-1);	% Desired position
p_d = [-0.75;-0.75];

% [~, p_d, p_o] = Functions.environment_setup(environment, dynamics, N_a);

%% Calculate Potential Field
U_att = zeros(length(x), length(y));
U_rep = zeros(length(x), length(y));
for i = 1:length(x)
    for j = 1:length(y)
        p = [x(i); y(j)];
        U_att(i,j) = 1/2*k_att*norm(p - p_d);

        for o = 1:N_o
            rho = norm(p - p_o(:,o)) - r_a - r_o;   
            if rho < rho_0
                rho = max(rho, 1e-6);           % In order to fill in the cylinder
                U_rep(i,j) = U_rep(i,j) + 1/2*k_rep*(1/rho - 1/rho_0)^2;
            end
        end
    end
end

Potential = U_att + U_rep;
Potential = min(Potential, 1*max(max(U_att)));
% Potential = U_att;
% Potential =  min(U_rep, 1*max(max(U_att)));

% Normalize the potential field to the range [0, 1]
Potential_min = min(Potential(:));  % Find the minimum value of the potential
Potential_max = max(Potential(:));  % Find the maximum value of the potential
Potential = (Potential - Potential_min) / (Potential_max - Potential_min);  % Remap to [0, 1]

fprintf('Potential Field Generated\n');

%% Calculate Vector Field
F_apf = zeros(length(x), length(y), m);
F_apfsf = zeros(length(x), length(y), m);
for i = 1:length(x)
    for j = 1:length(y)
        p = [x(i); y(j)];
        [gradU_att, gradU_rep, h] = Functions.potential_gradients(m, p, p_d, p_o, k_att, k_rep, r_a, r_o, rho_0);
        F_att = -gradU_att; 
        F_rep = -gradU_rep;
        F_total = F_att + F_rep;
        F_apf(i,j,:) = F_total/norm(F_total);

        sigma = norm(F_att)^2;
        gamma = k_gamma*norm(F_rep)^2 + gamma_static;
        alpha = k_alpha*min(h);
        [F_att, F_rep] = Functions.APF_safety_filter(m, F_att, F_rep, sigma, gamma, alpha);
        F_total = F_att + F_rep;
        F_apfsf(i,j,:) = F_total/norm(F_total);
    end
end
fprintf('Vector Field Generated\n');

%% Plot results
load('+Functions\customColors.mat');
close all
figure('Position', [100 50 730 700]);   %Left Bottom Width Height
% figure('Position', [100 50 820 500]);  %SD
hold on; grid on;
% surf(x, y, Potential','FaceAlpha',1, 'EdgeColor','none')
% contour(x, y, Potential', 20, 'LineWidth', 1.5);
quiver(x, y, squeeze(F_apf(:,:,1))', squeeze(F_apf(:,:,2))', 0.5, 'Color', DesmosColors(2,:), 'LineWidth', 1.5);
quiver(x, y, squeeze(F_apfsf(:,:,1))', squeeze(F_apfsf(:,:,2))', 0.5, 'Color', DesmosColors(1,:), 'LineWidth', 1.5);
% plot(x,y)
% clim([0 1]);  % Ensure color scale goes from 0 to 1
% cb = colorbar;
% cb.Ticks = linspace(0, 1, 5); % Set colorbar ticks evenly
% cb.TickLabels = linspace(0, 1, 5); % Override labels from 0 to 1
% cb.TickLabelInterpreter = 'latex';
% colormap('parula');
ax = gca; 
% ax.ZTick = linspace(0, 1, 5); % Set z-axis ticks evenly
% ax.ZTickLabel = linspace(0, 1, 5); % Override z-axis labels to match [0, 1]
set(ax, 'FontSize', 22); ax.TickLabelInterpreter = 'latex';
% ax.XTickLabel = linspace(-1, 1, 5);
% ax.YTickLabel = linspace(-1, 1, 5);
ax.XTick = linspace(-1, 1, 5);
ax.YTick = linspace(-1, 1, 5);

% % Plot obstacles
grey = DesmosColors(6,:);
th = 0:pi/50:2*pi;
for j = 1:N_o
    x_obs = r_o * cos(th) + p_o(1,j);
    y_obs = r_o * sin(th) + p_o(2,j);
    patch(x_obs, y_obs, grey,'FaceColor', grey, ...
                        'FaceAlpha', 1,...
                        'EdgeColor', 'black', ...
                        'HandleVisibility', 'off');
    % text(p_o(1,j)-0.2*r_o, p_o(2,j)-0.027*r_o, sprintf('%d', j), ...
    %                                  'Color', 'white', ...
    %                                  'Interpreter','latex', ...
    %                                  'FontSize', 25);
end
% Plot desired position
plot(p_d(1), p_d(2), 'x','MarkerSize', 30, ...
                         'MarkerEdgeColor', 'red', ...
                         'LineWidth', 6,...
                         'HandleVisibility', 'off');

% Setup
axis('equal')
xlim([rangeX(1) rangeX(2)]); ylim([rangeY(1) rangeY(2)]); 
% zlim([0 1]);
xlabel_handle = xlabel('$x_1$', 'Interpreter','latex', 'FontSize', 32);
ylabel_handle = ylabel('$x_2$', 'Interpreter','latex', 'FontSize', 32);

xlabel_handle.Position(2) = xlabel_handle.Position(2) + 0.075;    % move the label 0.3 data-units further up
ylabel_handle.Position(1) = ylabel_handle.Position(1) + 0.08;    % move the label 0.3 data-units further up


% ylabel(cb,'$U_{{\scriptscriptstyle \!A\!P\!F}}(\mathbf{x})$', 'Interpreter','latex', 'FontSize', 34,'Rotation',270);
% zlabel('$U_{tot}(\mathbf{x})$', 'Interpreter','latex', 'FontSize', 28);
% title('Repulsive Potential Function', 'Interpreter', 'latex', 'FontSize', 22);
view(-15, 45);
view(0, 90);
