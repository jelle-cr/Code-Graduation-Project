function plot_animated_trajectories(rangeX, rangeY, plottingFolder)

    if nargin == 0
        close all
        rangeX = [-3; 3];
        rangeY = [-2; 2];
        % Place a Parameter file, along with all of the simulation outputs in the plottingFolder
        plottingFolder = 'Data';

        % Create the VideoWriter object
        timestamp = datestr(now, 'dd-mm_HH-MM-SS');
        videoFileName = ['Videos/Simulations/animated_trajectory_' timestamp '.mp4'];  % Output file name
        vidObj = VideoWriter(videoFileName, 'MPEG-4');              % 'MPEG-4' for .mp4 format
        vidObj.FrameRate = 30;                                      % Set the frame rate (frames per second)
        open(vidObj);                                               % Open the video file for writing
    end
    
    %% Loading data (only loads a single file)
    % Get a list of all .mat files in the plottingFolder
    fileList = dir(fullfile(plottingFolder, '*.mat'));

    % Select first file and load it
    data = fullfile(fileList(1).folder, fileList(1).name);

    % % Override filename
    % data = './Data/SimulationDataRecent.mat';

    load(data, 'x_0', 'x_d', 'x_o', 'r_a', 'N_a', 'r_o', 'N_o', 't_step', 't_end');

    quality = 'SD';
    % quality = 'HD';     % High definition
    legendHandle = Functions.trajectory_plot_setup(rangeX, rangeY, x_0, x_d, x_o, N_a, r_o, N_o, quality);      % Sets up obstacles, initial and desired positions, and legend

    %% Load multiple file data
    if length(fileList) > 1
        N_a = length(fileList);
        i = 1;
        for k = length(fileList):-1:1
        % for k = 1:length(fileList)
            % Load desired datasets
            dataFile = fullfile(fileList(k).folder, fileList(k).name);
            fprintf('Processing file: %s\n', dataFile);
            load(dataFile, 'x');
            x_temp(:,i,:) = x;
            i = i + 1;
        end
        x = x_temp;
    end

    %% Manually set legend
    load('+Functions\customColors.mat');
    colors = DesmosColors;

    existingHandles = legendHandle.PlotChildren;
    existingLabels = legendHandle.String;

    newHandles = [];
    for i = 1:N_a
        newHandles(i) = scatter(NaN, NaN, 36, 'o', 'filled', 'MarkerEdgeColor', colors(i,:), ...
                                        'MarkerFaceColor', colors(i,:), ...
                                        'MarkerFaceAlpha', 0.5);
    end
    newLabels = [{'hi','bye'}];

    % Append new handles and labels
    % updatedHandles = [existingHandles; newHandles]; % Combine new and existing handles
    updatedLabels = [existingLabels, newLabels];    % Combine new and existing labels

    % Update the legend
    % legend(updatedHandles, updatedLabels);

    %% Trajectory plotting    
    th = 0:pi/50:2*pi;                    % Angles for creating the circle shape
         
    p = x(1:2,:,:);
    agentHandles = zeros(N_a, 1);         % Handles for agents
    
    for t_ind = 1:(t_end/t_step + 1)
        for i = 1:N_a
            x_agent = r_a * cos(th) + p(1,i,t_ind);
            y_agent = r_a * sin(th) + p(2,i,t_ind);
            if agentHandles(i) == 0
                agentHandles(i) = patch(x_agent, y_agent, colors(i,:), ...
                                        'FaceColor', colors(i,:), ...
                                        'FaceAlpha', 0.2, ...
                                        'EdgeColor', colors(i,:), ...
                                        'HandleVisibility', 'off');
                                        % 'DisplayName', [sprintf('Agent %d', i), '\hspace{1.2mm}']);
                % uistack(agentHandles(i), 'bottom');
            else
                set(agentHandles(i), 'XData', x_agent, ...
                                     'YData', y_agent);
            end
        end
        drawnow;
        
        if nargin == 0
            % Capture the current frame and write it to the video
            frame = getframe(gcf);  % Capture the current figure
            writeVideo(vidObj, frame);  % Write the captured frame to the video
        end
    end
    
    % Save the video if this function was run explicitly
    if nargin == 0
        % Close the video writer
        close(vidObj);  % Finalize and save the video
        fprintf('Video Saved\n');
    end
end

