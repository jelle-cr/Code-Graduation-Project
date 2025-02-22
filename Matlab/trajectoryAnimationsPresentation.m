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
   
quality = 'HD'; % Definition
rangeX = [-5; 5];
rangeY = [-3; 3];
[~] = Functions.trajectory_plot_setup(rangeX, rangeY, x_0, x_d, x_o, N_a, r_o, N_o, quality);      % Sets up obstacles, initial and desired positions, and legend
ax = gca;
ax.XTick = -5:5; % Set tick positions
ax.XTickLabel = {'', '', '', '', '', '', '', '', '', '', ''}; % Set custom tick labels
ax.YTick = -3:3; % Set tick positions
ax.YTickLabel = {'', '', '', '', '', '', ''}; % Set custom tick labels

load('+Functions\customColors.mat');
colors = DesmosColors;
minColor = colors(2, :); % Minimum color
maxColor = [1,1,1]%colors(1, :); % Maximum color

% Create a custom colormap with a smooth gradient between minColor and maxColor
customColormap = [linspace(minColor(1), maxColor(1), 256)', ...
                  linspace(minColor(2), maxColor(2), 256)', ...
                  linspace(minColor(3), maxColor(3), 256)'];
colormap(customColormap);

%% Extra plot options (don't use for varying h_0 plot)
k_gamma = 0;
k_alpha = 1;
rho_0 = 1;

% Plot repulsive region for Safety Filter
% num_x_steps = 450;
% num_y_steps = 750;
num_x_steps = 900;
num_y_steps = 1500;
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

% Plot rho_0
th = 0:pi/50:2*pi;
for i = 1:N_o
    x_rho_0 = (r_o + rho_0) * cos(th) + x_o(1,i);
    y_rho_0 = (r_o + rho_0) * sin(th) + x_o(2,i);
    % % patch(x_rho_0, y_rho_0, 'b', 'FaceAlpha', 0, 'EdgeColor', colors(2,:), 'LineWidth', 2);
    plot(x_rho_0, y_rho_0, '--', 'Color', colors(6,:), 'LineWidth', 2);
end

%%  Plot one dataset at a time manually (e.g. uncomment and run one by one)
% Single integrator varying rho_0 (rho_0 = h_0)
% load([plottingFolder, '/SI-APF-N_o3-rho10-k_rep1.mat']); k=2;
% load([plottingFolder, '/SI-SF-N_o3-rho10-k_gamma0-k_alpha1-k_rep1.mat']); k=1;
% load([plottingFolder, '/SI-APF-N_o3-rho0.05-k_rep1.mat']); k=2;
% load([plottingFolder, '/SI-SF-N_o3-rho0.05-k_gamma0-k_alpha1-k_rep1.mat']); k=1;   

% Single integrator varying initial position
% load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1-x_0-4_2.mat']); k=2;
% load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1-x_0-4_2.mat']); k=5;
% load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1-x_0-4_0.mat']); k=2;
% load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1-x_0-4_0.mat']); k=1;
% load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1-x_0-4_-2.mat']); k=2;
% load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1-x_0-4_-2.mat']); k=1;
% load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1-x_00_-2.mat']); k=2;
% load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1-x_00_-2.mat']); k=1;

% Double integrator varying k_pid (proportional feedback gain)
% load([plottingFolder, '/DI-APF-N_o3-rho1-k_rep1-k_pid1.mat']); k=2;
% load([plottingFolder, '/DI-SF-N_o3-rho1-k_gamma1-k_alpha1-k_rep1-k_pid1']); k=1;
% load([plottingFolder, '/DI-APF-N_o3-rho1-k_rep1-k_pid5.mat']); k=2;
% load([plottingFolder, '/DI-SF-N_o3-rho1-k_gamma1-k_alpha1-k_rep1-k_pid5']); k=1;
% load([plottingFolder, '/DI-APF-N_o3-rho1-k_rep1-k_pid10.mat']); k=2;
% load([plottingFolder, '/DI-SF-N_o3-rho1-k_gamma1-k_alpha1-k_rep1-k_pid10']); k=1;

%% Single integrator
plottingFolder = 'Data/SIvarying_x_0';
load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1-x_0-4_2.mat']);
p_apf_1 = squeeze(x(1:2,1,:));
u_apf_1 = squeeze(u_att + u_rep);
load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1-x_0-4_2.mat']);
p_sf_1 = squeeze(x(1:2,1,:));
u_sf_1 = squeeze(u_att + u_rep);
load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1-x_0-4_0.mat']);
p_apf_2 = squeeze(x(1:2,1,:));
u_apf_2 = squeeze(u_att + u_rep);
load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1-x_0-4_0.mat']);
p_sf_2 = squeeze(x(1:2,1,:));
u_sf_2 = squeeze(u_att + u_rep);
load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1-x_0-4_-2.mat']);
p_apf_3 = squeeze(x(1:2,1,:));
u_apf_3 = squeeze(u_att + u_rep);
load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1-x_0-4_-2.mat']);
p_sf_3 = squeeze(x(1:2,1,:));
u_sf_3 = squeeze(u_att + u_rep);
load([plottingFolder, '/SI-APF-N_o3-rho1-k_rep1-x_00_-2.mat']);
p_apf_4 = squeeze(x(1:2,1,:));
u_apf_4 = squeeze(u_att + u_rep);
load([plottingFolder, '/SI-SF-N_o3-rho1-k_gamma0-k_alpha1-k_rep1-x_00_-2.mat']);
p_sf_4 = squeeze(x(1:2,1,:));
u_sf_4 = squeeze(u_att + u_rep);

%% Double integrator 
% plottingFolder = 'Data/DIvarying_k_pid';
% load([plottingFolder, '/DI-APF-N_o3-rho1-k_rep1-k_pid1.mat']);
% p_apf_1 = squeeze(x(1:2,1,:));
% load([plottingFolder, '/DI-SF-N_o3-rho1-k_gamma1-k_alpha1-k_rep1-k_pid1']);
% p_sf_1 = squeeze(x(1:2,1,:));
% load([plottingFolder, '/DI-APF-N_o3-rho1-k_rep1-k_pid5.mat']);
% p_apf_2 = squeeze(x(1:2,1,:));
% load([plottingFolder, '/DI-SF-N_o3-rho1-k_gamma1-k_alpha1-k_rep1-k_pid5']);
% p_sf_2 = squeeze(x(1:2,1,:));
% load([plottingFolder, '/DI-APF-N_o3-rho1-k_rep1-k_pid10.mat']);
% p_apf_3 = squeeze(x(1:2,1,:));
% load([plottingFolder, '/DI-SF-N_o3-rho1-k_gamma1-k_alpha1-k_rep1-k_pid10']);
% p_sf_3 = squeeze(x(1:2,1,:));

           
%% Plotting
% Create the VideoWriter object
% timestamp = datestr(now, 'dd-mm_HH-MM-SS');
% videoFileName = ['Videos/animated_trajectory_' timestamp '.mp4'];  % Output file name
% vidObj = VideoWriter(videoFileName, 'MPEG-4');              % 'MPEG-4' for .mp4 format
% vidObj.FrameRate = 30;                                      % Set the frame rate (frames per second)
% open(vidObj);                                               % Open the video file for writing

apfCol = colors(2,:);
sfCol = colors(5,:);
dotSize = 75;

t_ind = 1;
for t = 0:10*t_step:t_end
    if t_ind == 1
        trajectoryHandlesAPF(1) = plot(p_apf_1(1,1:t_ind), p_apf_1(2,1:t_ind), 'Color', apfCol,...
                                                        'LineWidth', 4,...
                                                        'LineStyle', '-',...
                                                        'HandleVisibility', 'off');
        trajectoryHandlesSF(1) = plot(p_sf_1(1,1:t_ind), p_sf_1(2,1:t_ind), 'Color', sfCol,...
                                                        'LineWidth', 4,...
                                                        'LineStyle', '-',...
                                                        'HandleVisibility', 'off');
        positionHandlesAPF(1) = plot(p_apf_1(1,t_ind), p_apf_1(2,t_ind), 'Color', apfCol,...
                                                        'Marker', '.',...
                                                        'MarkerSize', dotSize,...
                                                        'HandleVisibility', 'off');
        positionHandlesSF(1) = plot(p_sf_1(1,t_ind), p_sf_1(2,t_ind), 'Color', sfCol,...
                                                        'Marker', '.',...
                                                        'MarkerSize', dotSize,...
                                                        'HandleVisibility', 'off');
        trajectoryHandlesAPF(2) = plot(p_apf_2(1,1:t_ind), p_apf_2(2,1:t_ind), 'Color', apfCol,...
                                                        'LineWidth', 4,...
                                                        'LineStyle', '-',...
                                                        'HandleVisibility', 'off');
        trajectoryHandlesSF(2) = plot(p_sf_2(1,1:t_ind), p_sf_2(2,1:t_ind), 'Color', sfCol,...
                                                        'LineWidth', 4,...
                                                        'LineStyle', '-',...
                                                        'HandleVisibility', 'off');
        positionHandlesAPF(2) = plot(p_apf_2(1,t_ind), p_apf_2(2,t_ind), 'Color', apfCol,...
                                                        'Marker', '.',...
                                                        'MarkerSize', dotSize,...
                                                        'HandleVisibility', 'off');
        positionHandlesSF(2) = plot(p_sf_2(1,t_ind), p_sf_2(2,t_ind), 'Color', sfCol,...
                                                        'Marker', '.',...
                                                        'MarkerSize', dotSize,...
                                                        'HandleVisibility', 'off');
        trajectoryHandlesAPF(3) = plot(p_apf_3(1,1:t_ind), p_apf_3(2,1:t_ind), 'Color', apfCol,...
                                                        'LineWidth', 4,...
                                                        'LineStyle', '-',...
                                                        'HandleVisibility', 'off');
        trajectoryHandlesSF(3) = plot(p_sf_3(1,1:t_ind), p_sf_3(2,1:t_ind), 'Color', sfCol,...
                                                        'LineWidth', 4,...
                                                        'LineStyle', '-',...
                                                        'HandleVisibility', 'off');
        positionHandlesAPF(3) = plot(p_apf_3(1,t_ind), p_apf_3(2,t_ind), 'Color', apfCol,...
                                                        'Marker', '.',...
                                                        'MarkerSize', dotSize,...
                                                        'HandleVisibility', 'off');
        positionHandlesSF(3) = plot(p_sf_3(1,t_ind), p_sf_3(2,t_ind), 'Color', sfCol,...
                                                        'Marker', '.',...
                                                        'MarkerSize', dotSize,...
                                                        'HandleVisibility', 'off');
        trajectoryHandlesAPF(4) = plot(p_apf_4(1,1:t_ind), p_apf_4(2,1:t_ind), 'Color', apfCol,...
                                                        'LineWidth', 4,...
                                                        'LineStyle', '-',...
                                                        'HandleVisibility', 'off');
        trajectoryHandlesSF(4) = plot(p_sf_4(1,1:t_ind), p_sf_4(2,1:t_ind), 'Color', sfCol,...
                                                        'LineWidth', 4,...
                                                        'LineStyle', '-',...
                                                        'HandleVisibility', 'off');
        positionHandlesAPF(4) = plot(p_apf_4(1,t_ind), p_apf_4(2,t_ind), 'Color', apfCol,...
                                                        'Marker', '.',...
                                                        'MarkerSize', dotSize,...
                                                        'HandleVisibility', 'off');
        positionHandlesSF(4) = plot(p_sf_4(1,t_ind), p_sf_4(2,t_ind), 'Color', sfCol,...
                                                        'Marker', '.',...
                                                        'MarkerSize', dotSize,...
                                                        'HandleVisibility', 'off');
    else
        set(trajectoryHandlesAPF(1), 'XData', p_apf_1(1,1:t_ind), ...
                                     'YData', p_apf_1(2,1:t_ind));
        set(trajectoryHandlesSF(1), 'XData', p_sf_1(1,1:t_ind), ...
                                    'YData', p_sf_1(2,1:t_ind));
        set(positionHandlesAPF(1), 'XData', p_apf_1(1,t_ind), ...
                                   'YData', p_apf_1(2,t_ind));
        set(positionHandlesSF(1), 'XData', p_sf_1(1,t_ind), ...
                                  'YData', p_sf_1(2,t_ind));
        set(trajectoryHandlesAPF(2), 'XData', p_apf_2(1,1:t_ind), ...
                                     'YData', p_apf_2(2,1:t_ind));
        set(trajectoryHandlesSF(2), 'XData', p_sf_2(1,1:t_ind), ...
                                    'YData', p_sf_2(2,1:t_ind));
        set(positionHandlesAPF(2), 'XData', p_apf_2(1,t_ind), ...
                                   'YData', p_apf_2(2,t_ind));
        set(positionHandlesSF(2), 'XData', p_sf_2(1,t_ind), ...
                                  'YData', p_sf_2(2,t_ind));
        set(trajectoryHandlesAPF(3), 'XData', p_apf_3(1,1:t_ind), ...
                                     'YData', p_apf_3(2,1:t_ind));
        set(trajectoryHandlesSF(3), 'XData', p_sf_3(1,1:t_ind), ...
                                    'YData', p_sf_3(2,1:t_ind));
        set(positionHandlesAPF(3), 'XData', p_apf_3(1,t_ind), ...
                                   'YData', p_apf_3(2,t_ind));
        set(positionHandlesSF(3), 'XData', p_sf_3(1,t_ind), ...
                                  'YData', p_sf_3(2,t_ind));
        set(trajectoryHandlesAPF(4), 'XData', p_apf_4(1,1:t_ind), ...
                                     'YData', p_apf_4(2,1:t_ind));
        set(trajectoryHandlesSF(4), 'XData', p_sf_4(1,1:t_ind), ...
                                    'YData', p_sf_4(2,1:t_ind));
        set(positionHandlesAPF(4), 'XData', p_apf_4(1,t_ind), ...
                                   'YData', p_apf_4(2,t_ind));
        set(positionHandlesSF(4), 'XData', p_sf_4(1,t_ind), ...
                                  'YData', p_sf_4(2,t_ind));


    end
               
                    
    drawnow;

    % frame = getframe(gcf);  % Capture the current figure
    % writeVideo(vidObj, frame);  % Write the captured frame to the video

    t_ind = t_ind + 10;
end

% close(vidObj);  % Finalize and save the video
% fprintf('############################### Video Saved ########################################\n');


saveas(gcf, 'Images/SItrajectoryEndInits.svg');

%% Control input
num_steps = t_end/t_step + 1;
for t_ind = 1:num_steps
        u_norm_apf_1(t_ind) = norm(squeeze(u_apf_1(:,t_ind)))^2;
        u_norm_sf_1(t_ind) = norm(squeeze(u_sf_1(:,t_ind)))^2;
        u_norm_apf_2(t_ind) = norm(squeeze(u_apf_2(:,t_ind)))^2;
        u_norm_sf_2(t_ind) = norm(squeeze(u_sf_2(:,t_ind)))^2;
        u_norm_apf_3(t_ind) = norm(squeeze(u_apf_3(:,t_ind)))^2;
        u_norm_sf_3(t_ind) = norm(squeeze(u_sf_3(:,t_ind)))^2;
        u_norm_apf_4(t_ind) = norm(squeeze(u_apf_4(:,t_ind)))^2;
        u_norm_sf_4(t_ind) = norm(squeeze(u_sf_4(:,t_ind)))^2;
end

lw = 2;
figure('Position', [100, 100, 1220, 800]);
% figure('Position', [100, 100, 1920, 1080]);
hold on; grid on; 
% plot(u_norm_apf_1, 'LineWidth',lw);
% plot(u_norm_sf_1, 'LineWidth',lw);
% plot(u_norm_apf_2, 'LineWidth',lw);
% plot(u_norm_sf_2, 'LineWidth',lw);
plot(u_norm_apf_3, 'LineWidth',lw, 'Color', apfCol);
plot(u_norm_sf_3, 'LineWidth',lw, 'Color', sfCol);
% plot(u_norm_apf_4, 'LineWidth',lw);
% plot(u_norm_sf_4, 'LineWidth',lw);
ylim([0, 80]);
xlim([0, 5000]);

%% Tracking error
num_steps = t_end/t_step + 1;
for t_ind = 1:num_steps
        % u_norm_apf_1(t_ind) = norm(squeeze(u_apf_1(:,t_ind)))^2;
        % u_norm_sf_1(t_ind) = norm(squeeze(u_sf_1(:,t_ind)))^2;
        % u_norm_apf_2(t_ind) = norm(squeeze(u_apf_2(:,t_ind)))^2;
        % u_norm_sf_2(t_ind) = norm(squeeze(u_sf_2(:,t_ind)))^2;
        e_apf_3(t_ind) = norm(p_d - p_apf_3(:,t_ind));
        e_sf_3(t_ind) = norm(p_d - p_sf_3(:,t_ind));
        % u_norm_apf_4(t_ind) = norm(squeeze(u_apf_4(:,t_ind)))^2;
        % u_norm_sf_4(t_ind) = norm(squeeze(u_sf_4(:,t_ind)))^2;
end

lw = 2;
figure('Position', [100, 100, 1220, 800]);
% figure('Position', [100, 100, 1920, 1080]);
hold on; grid on; 
% plot(u_norm_apf_1, 'LineWidth',lw);
% plot(u_norm_sf_1, 'LineWidth',lw);
% plot(u_norm_apf_2, 'LineWidth',lw);
% plot(u_norm_sf_2, 'LineWidth',lw);
plot(e_apf_3, 'LineWidth',lw, 'Color', apfCol);
plot(e_sf_3, 'LineWidth',lw, 'Color', sfCol);
% plot(u_norm_apf_4, 'LineWidth',lw);
% plot(u_norm_sf_4, 'LineWidth',lw);
% ylim([0, 80]);
xlim([0, 5000]);