
close all
clear all

rangeX = [-1; 1];
rangeY = [-1; 1];
num_steps = 17;
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

k_att = 1;
k_rep = 0.1;
rho_0 = 0.5; 
p_o = [0;
       0];

r_a = 0;                  % Radius of agent
r_o = 0.395;
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
        if norm(p) > r_o        % No vectors in obstacle
            [gradU_att, gradU_rep, h] = Functions.potential_gradients(m, p, p_d, p_o, k_att, k_rep, r_a, r_o, rho_0);
            F_att = -gradU_att; 
            F_rep = -gradU_rep;
            F_total = F_att + F_rep;
            F_apf(i,j,:) = F_total/norm(F_total);
    
            sigma = norm(F_att)^2;
            gamma = k_gamma*norm(F_rep)^2;
            alpha = k_alpha*min(h);
            [F_att, F_rep] = Functions.APF_safety_filter(m, F_att, F_rep, sigma, gamma, alpha);
            F_total = F_att + F_rep;
            F_apfsf(i,j,:) = F_total/norm(F_total);
        end
    end
end
fprintf('Vector Field Generated\n');

%% Plot repulsive region
num_steps = 300;
x2 = linspace(1.1*rangeX(1), 1.1*rangeX(2), num_steps);
y2 = linspace(1.1*rangeY(1), 1.1*rangeY(2), num_steps);

safeRegionSF = -2*ones(length(x2), length(y2));
safeRegionAPF = -2*ones(length(x2), length(y2));
for i = 1:length(x2)
    for j = 1:length(y2)
        p = [x2(i); y2(j)];
        if norm(p) > r_o        % No field in obstacle
            [gradU_att, gradU_rep, h] = Functions.potential_gradients(m, p, p_d, p_o, k_att, k_rep, r_a, r_o, rho_0);
            F_att = -gradU_att; 
            F_rep = -gradU_rep;
            sigma = norm(F_att)^2;
            gamma = k_gamma*norm(F_rep)^2;
            alpha = k_alpha*min(h);
            [~, ~, K_rep] = Functions.APF_safety_filter(m, F_att, F_rep, sigma, gamma, alpha);
            if K_rep <= 0
                safeRegionSF(i,j) = -1;
            end
            if norm(p) > r_o + rho_0
                safeRegionAPF(i,j) = -1;
            end
        end
    end
end
fprintf('Safe Region Generated\n');

%% Colorbar colors
load('+Functions\customColors.mat');
colors = DesmosColors;
minColor = colors(2, :); % Minimum color
maxColor = [1,1,1]%colors(5, :); % Maximum color

% Create a custom colormap with a smooth gradient between minColor and maxColor
customColormap = [linspace(minColor(1), maxColor(1), 256)', ...
                  linspace(minColor(2), maxColor(2), 256)', ...
                  linspace(minColor(3), maxColor(3), 256)'];

%% Plot results
close all
figure('Position', [100 50 730 700]);   %Left Bottom Width Height
% figure('Position', [100 50 820 500]);  %SD
hold on; grid on;
% surf(x, y, Potential','FaceAlpha',1, 'EdgeColor','none')
% surf(x2, y2, safeRegionAPF','FaceAlpha',0.25, 'EdgeColor','none');
th = 0:pi/50:2*pi;
x_rho_0 = (r_o + rho_0) * cos(th) + p_o(1,1);
y_rho_0 = (r_o + rho_0) * sin(th) + p_o(2,1);

imagesc(x2, y2, safeRegionSF','AlphaData',0.25);
plot(x_rho_0, y_rho_0, '--', 'Color', colors(6,:), 'LineWidth', 4);
quiver(x, y, squeeze(F_apf(:,:,1))', squeeze(F_apf(:,:,2))', 0.5, 'Color', DesmosColors(2,:), 'LineWidth', 1.7);
% quiver(x, y, squeeze(F_apfsf(:,:,1))', squeeze(F_apfsf(:,:,2))', 0.5, 'Color', DesmosColors(5,:), 'LineWidth', 2);


% Plot obstacles
grey = DesmosColors(6,:);
x_obs = r_o * cos(th) + p_o(1,1);
y_obs = r_o * sin(th) + p_o(2,1);
obs = patch(x_obs, y_obs, grey,'FaceColor', grey, ...
                        'FaceAlpha', 1,...
                        'EdgeColor', 'black', ...
                        'HandleVisibility', 'off');

% Plot desired position
des = plot(p_d(1), p_d(2), 'x','MarkerSize', 30, ...
                         'MarkerEdgeColor', 'red', ...
                         'LineWidth', 6,...
                         'HandleVisibility', 'off');

% Setup
colormap(customColormap);
ax = gca; 
set(ax, 'FontSize', 22); ax.TickLabelInterpreter = 'latex';                 % Small plots, 22
ax.XTick = linspace(-1, 1, 5);
ax.YTick = linspace(-1, 1, 5);
xlim([rangeX(1) rangeX(2)]); ylim([rangeY(1) rangeY(2)]); 
xlabel_handle = xlabel('$x_1$', 'Interpreter','latex', 'FontSize', 32);     % Small plots, 32    
ylabel_handle = ylabel('$x_2$', 'Interpreter','latex', 'FontSize', 32);
xlabel_handle.Position(2) = xlabel_handle.Position(2) + 0.055;              % Small plots, 0.055  
ylabel_handle.Position(1) = ylabel_handle.Position(1) + 0.11;               % Small plots, 0.011  
xlim([-1, 1]);
ylim([-1, 1]);
% axis('equal')