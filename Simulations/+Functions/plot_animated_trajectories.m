function plot_animated_trajectories(rangeX, rangeY, plottingFolder)
    close all

    if nargin == 0
        rangeX = [-3; 3];
        rangeY = [-2; 2];
        % Place a Parameter file, along with all of the simulation outputs in the plottingFolder
        plottingFolder = 'Data';

        % Create the VideoWriter object
        timestamp = datestr(now, 'dd-mm_HH-MM-SS');
        videoFileName = ['Videos/Simulations/animated_trajectory_' timestamp '.mp4'];  % Output file name
        vidObj = VideoWriter(videoFileName, 'MPEG-4');              % 'MPEG-4' for .mp4 format
        vidObj.FrameRate = 60;                                      % Set the frame rate (frames per second)
        % vidObj.Quality = 10;                                        % Set the video quality (0-100, higher is better)
        open(vidObj);                                               % Open the video file for writing
    end

    params = [plottingFolder '/Parameters.mat'];
    Functions.trajectory_plot_setup(rangeX, rangeY, params);      % Sets up obstacles, initial and desired positions, and legend
    
    %% Loading data
    load(params);
    load('Data/SimulationDataRecent.mat');

    % % Get a list of all .mat files in the plottingFolder
    % fileList = dir(fullfile(plottingFolder, '*.mat'));
    % % Exclude 'Parameters.mat' from the list
    % fileList = fileList(~strcmp({fileList.name}, 'Parameters.mat'));
    % dataFile = fullfile(fileList(1).folder, fileList(1).name);
    % fprintf('Processing file: %s\n', dataFile);
    % load(dataFile);

    %% Trajectory plotting    
    colors = lines(N_a);                  % All agents different color -> colors(i,:)
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

