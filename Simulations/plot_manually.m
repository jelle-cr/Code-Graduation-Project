close all
clear all

plottingFolder = 'Data/SIvaryingRho';

% Get a list of all .mat files in the plottingFolder
fileList = dir(fullfile(plottingFolder, '*.mat'));

% Select first file and load specific parameters
params = fullfile(fileList(1).folder, fileList(1).name);
load(params, 'x_0', 'x_d', 'x_o', 'N_a', 'r_o', 'N_o', 't_step', 't_end');
   
quality = 'SD'; % Standard Definition
rangeX = [-5; 5];
rangeY = [-3; 3];
[~] = Functions.trajectory_plot_setup(rangeX, rangeY, x_0, x_d, x_o, N_a, r_o, N_o, quality);      % Sets up obstacles, initial and desired positions, and legend

load('+Functions\customColors.mat');
colors = DesmosColors;

%%  Plot one dataset at a time
N_p = 6;
% colors = lines(N_p);
load([plottingFolder, '/SI-APF-N_o3-rho10-k_rep1.mat']); k=2;
% load([plottingFolder, '/SI-SF-N_o3-rho10-k_gamma0-k_alpha1-k_rep1.mat']); k=1;
% load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1.mat']); k=2;
% load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1.mat']); k=1;
% load([plottingFolder, '/SI-APF-N_o3-rho0.05-k_rep1.mat']); k=2;
% load([plottingFolder, '/SI-SF-N_o3-rho0.05-k_gamma0-k_alpha1-k_rep1.mat']); k=1;   
           
trajectoryHandles = gobjects(N_a, 1); 

timepointInterval = 0.5;                  % Plots a point every 0.5 s
timepointHandles = gobjects(N_a, 1); 
t = 0:t_step:t_end;
tp_ind = mod(t/timepointInterval,1) == 0;
    
p = x(1:2,:,:);
    
for i = 1:N_a
    col = colors(k,:);
    trajectoryHandles(i) = plot(squeeze(p(1,i,:)), squeeze(p(2,i,:)), 'Color', col,...
                                        'LineWidth', 2,...
                                        'LineStyle', '-',...
                                        'DisplayName', 'temp');
                                        % 'DisplayName', [sprintf('Agent %d', i), '\hspace{1.2mm}']);
    timepointHandles(i) = scatter(squeeze(p(1,i,tp_ind)),squeeze(p(2,i,tp_ind)), 350, ...
                                        'Marker', '.',...
                                        'MarkerEdgeColor', col,...
                                        'HandleVisibility','off');
            
                % uistack(timepointHandles(i), 'bottom');
                % uistack(trajectoryHandles(i), 'bottom');
end