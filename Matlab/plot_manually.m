close all
clear all

plottingFolder = 'Data/SIvarying_h_0';
plottingFolder = 'Data/SIvarying_x_0';
% plottingFolder = 'Data/DIvarying_k_pid';

% Get a list of all .mat files in the plottingFolder
fileList = dir(fullfile(plottingFolder, '*.mat'));

% Select first file and load specific parameters
params = fullfile(fileList(1).folder, fileList(1).name);
load(params);
   
quality = 'SD'; % Standard Definition
rangeX = [-5; 5];
rangeY = [-3; 3];
[~] = Functions.trajectory_plot_setup(rangeX, rangeY, x_0, x_d, x_o, N_a, r_o, N_o, quality);      % Sets up obstacles, initial and desired positions, and legend

load('+Functions\customColors.mat');
colors = DesmosColors;
minColor = colors(2, :); % Minimum color
maxColor = colors(5, :); % Maximum color

% Create a custom colormap with a smooth gradient between minColor and maxColor
customColormap = [linspace(minColor(1), maxColor(1), 256)', ...
                  linspace(minColor(2), maxColor(2), 256)', ...
                  linspace(minColor(3), maxColor(3), 256)'];
colormap(customColormap);

%% Extra plot options (don't use for varying h_0 plot)
k_gamma = 1;
k_alpha = 1;
rho_0 = 1;

% Plot repulsive region for Safety Filter
num_x_steps = 450;
num_y_steps = 750;
x2 = linspace(1.1*rangeX(1), 1.1*rangeX(2), num_x_steps);
y2 = linspace(1.1*rangeY(1), 1.1*rangeY(2), num_y_steps);

safeRegionSF = zeros(length(x2), length(y2));

p_d = x_d(1:2);
p_o = x_o(1:2,:);
for i = 1:length(x2)
    for j = 1:length(y2)
        p_a = [x2(i); y2(j)];
        [gradU_att, gradU_rep, h] = Functions.potential_gradients(m, p_a, p_d, p_o, k_att, k_rep, r_a, r_o, rho_0);
        F_att = -gradU_att; 
        F_rep = -gradU_rep;
        sigma = norm(F_att)^2;
        gamma = k_gamma*norm(F_rep)^2;
        alpha = k_alpha*min(h);
        [~, ~, K_rep] = Functions.APF_safety_filter(m, F_att, F_rep, sigma, gamma, alpha);
        if K_rep <= 0
            safeRegionSF(i,j) = 1;
        end
    end
end
regions = imagesc(x2, y2, safeRegionSF','AlphaData',0.25);
uistack(regions, 'bottom');

% Plot extra initial positions
initPos = [-4, -4, -4, 0;
           -2,  0,  2, -2];
for i = 1:4
    plot(initPos(1,i), initPos(2,i), 'o','MarkerSize', 1*20, ...
                                                       'MarkerEdgeColor', 'blue', ...
                                                       'LineWidth', 1*4,...
                                                       'HandleVisibility', 'off');
end

% Plot rho_0
th = 0:pi/50:2*pi;
for i = 1:N_o
    x_rho_0 = (r_o + rho_0) * cos(th) + x_o(1,i);
    y_rho_0 = (r_o + rho_0) * sin(th) + x_o(2,i);
    % patch(x_rho_0, y_rho_0, 'b', 'FaceAlpha', 0, 'EdgeColor', colors(2,:), 'LineWidth', 2);
    plot(x_rho_0, y_rho_0, '--', 'Color', colors(6,:), 'LineWidth', 2);
end

%%  Plot one dataset at a time manually (e.g. uncomment and run one by one)
% Single integrator varying rho_0 (rho_0 = h_0)
% load([plottingFolder, '/SI-APF-N_o3-rho10-k_rep1.mat']); k=2;
% load([plottingFolder, '/SI-SF-N_o3-rho10-k_gamma0-k_alpha1-k_rep1.mat']); k=1;
% load([plottingFolder, '/SI-APF-N_o3-rho0.05-k_rep1.mat']); k=2;
% load([plottingFolder, '/SI-SF-N_o3-rho0.05-k_gamma0-k_alpha1-k_rep1.mat']); k=1;   

% Single integrator varying initial position
load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1-x_0-4_2.mat']); k=2;
load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1-x_0-4_2.mat']); k=1;
load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1-x_0-4_0.mat']); k=2;
load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1-x_0-4_0.mat']); k=1;
load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1-x_0-4_-2.mat']); k=2;
load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1-x_0-4_-2.mat']); k=1;
load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1-x_00_-2.mat']); k=2;
load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1-x_00_-2.mat']); k=1;

% Double integrator varying k_pid (proportional feedback gain)
% load([plottingFolder, '/DI-APF-N_o3-rho1-k_rep1-k_pid1.mat']); k=2;
% load([plottingFolder, '/DI-SF-N_o3-rho1-k_gamma1-k_alpha1-k_rep1-k_pid1']); k=1;
% load([plottingFolder, '/DI-APF-N_o3-rho1-k_rep1-k_pid5.mat']); k=2;
% load([plottingFolder, '/DI-SF-N_o3-rho1-k_gamma1-k_alpha1-k_rep1-k_pid5']); k=1;
% load([plottingFolder, '/DI-APF-N_o3-rho1-k_rep1-k_pid10.mat']); k=2;
% load([plottingFolder, '/DI-SF-N_o3-rho1-k_gamma1-k_alpha1-k_rep1-k_pid10']); k=1;

u_norm_avg
           
trajectoryHandles = gobjects(N_a, 1); 

timepointInterval = 0.5;                  % Plots a point every 0.5 s
timepointHandles = gobjects(N_a, 1); 
t = 0:t_step:t_end;
tp_ind = mod(t/timepointInterval,1) == 0;
    
p = x(1:2,:,:);
    
for i = 1:N_a
    col = colors(k,:);
    trajectoryHandles(i) = plot(squeeze(p(1,i,:)), squeeze(p(2,i,:)), 'Color', col,...
                                        'LineWidth', 1,...
                                        'LineStyle', '-',...
                                        'DisplayName', 'temp');
    % timepointHandles(i) = scatter(squeeze(p(1,i,tp_ind)),squeeze(p(2,i,tp_ind)), 350, ...
    %                                     'Marker', '.',...
    %                                     'MarkerEdgeColor', col,...
    %                                     'HandleVisibility','off');
end