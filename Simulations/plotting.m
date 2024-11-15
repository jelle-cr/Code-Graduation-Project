close all
clear all
clc
rangeX = [-3; 3];
rangeY = [-2; 2];
load('Data/Parameters.mat');
t = 0:t_step:t_end; 
p_d = x_d(1:2);
p_o = x_o(1:2,:);

%% Setup plot
figure('Position', [100 50 1000 500]);  %Left Bottom Width Height
hold on; grid on; axis equal; 
ax = gca; set(ax, 'FontSize', 22); ax.TickLabelInterpreter = 'latex';
xlim(rangeX); ylim(rangeY);
xticks(rangeX(1):1:rangeX(2)); yticks(rangeY(1):1:rangeY(2));
xlabel('$x_1$ [m]', 'Interpreter','latex', 'FontSize', 30);
ylabel('$x_2$ [m]', 'Interpreter','latex', 'FontSize', 30);

%% Create legend entries
legend('Location', 'northeastoutside', 'BackgroundAlpha', 0.7, 'Interpreter', 'latex', 'FontSize', 26);

% Create initial positions in legend
plot(NaN, NaN, 'o', 'MarkerSize', 15, ...
                    'MarkerEdgeColor', 'blue', ...
                    'LineWidth', 4, ...
                    'DisplayName', 'Initial Position');
% Create goal in legend
plot(NaN, NaN, 'x', 'MarkerSize', 20, ...
                    'MarkerEdgeColor', 'red', ...
                    'LineWidth', 4,...
                    'DisplayName', 'Desired Position');

%% Plot obstacles
plot(NaN, NaN, 'o', 'MarkerEdgeColor', 'black', ...
                    'MarkerFaceColor', '#9aa3b3', ...
                    'MarkerSize', 15, ...
                    'DisplayName', 'Obstacles');
th = 0:pi/50:2*pi;
rgbColor = hex2rgb('#9aa3b3');
for j = 1:N_o
    x_obs = r_o * cos(th) + p_o(1,j);
    y_obs = r_o * sin(th) + p_o(2,j);
    patch(x_obs, y_obs, rgbColor,'FaceColor', rgbColor, ...
                        'EdgeColor', 'black', ...
                        'HandleVisibility', 'off');
    text(p_o(1,j)-0.11*r_o, p_o(2,j)-0.027*r_o, sprintf('%d', j), ...
                                     'Color', 'white', ...
                                     'Interpreter','latex', ...
                                     'FontSize', 25);
end

%% Trajectory plotting
% Load desired datasets
load('Data/Parameters.mat');
load('Data/SimulationData.mat');
colors = lines(N_a);  

p = x(1:2,:,:);
for i = 1:N_a
    plot(squeeze(p(1,i,:)), squeeze(p(2,i,:)), 'Color', colors(i,:),...
                            'LineWidth', 4,...
                            'LineStyle', '-',...
                         'DisplayName', '$\rho_0=0.15$');
                         % 'HandleVisibility', 'off');
                         % 'DisplayName', [sprintf('Agent %d', i), '\hspace{1.2mm}']);
end

%% Static plotting (goes on top of trajectory)
% Plot initial positions
for i = 1:N_a
    plot(squeeze(p(1,i,1)), squeeze(p(2,i,1)), 'o','MarkerSize', 20, ...
                                                   'MarkerEdgeColor', 'blue', ...
                                                   'LineWidth', 4,...
                                                   'HandleVisibility', 'off');
end 
% Plot Goal
plot(p_d(1), p_d(2), 'x','MarkerSize', 30, ...
                         'MarkerEdgeColor', 'red', ...
                         'LineWidth', 6,...
                         'HandleVisibility', 'off');
