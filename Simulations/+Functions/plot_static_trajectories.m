function plot_static_trajectories(rangeX, rangeY, plottingFolder)
    if nargin == 0
        rangeX = [-3; 3];
        rangeY = [-2; 2];
        % Place a Parameter file, along with all of the simulation outputs in the plottingFolder
        plottingFolder = 'Data/plottingData';
    end

    params = [plottingFolder '/Parameters.mat'];
    Functions.trajectory_plot_setup(rangeX, rangeY, params);      % Sets up obstacles, initial and desired positions, and legend
    
    
    %% Trajectory plotting
    
    % Get a list of all .mat files in the plottingFolder
    fileList = dir(fullfile(plottingFolder, '*.mat'));
    
    % Exclude 'Parameters.mat' from the list
    fileList = fileList(~strcmp({fileList.name}, 'Parameters.mat'));
    
    if ~isempty(fileList)
        for k = 1:length(fileList)
            % Load desired datasets
            dataFile = fullfile(fileList(k).folder, fileList(k).name);
            fprintf('Processing file: %s\n', dataFile);
            load(dataFile);
    
            if N_a > 1
                colors = lines(N_a);                  % All agents different color -> colors(i,:)
            else
                colors = lines(length(fileList));       % All simulations different color -> colors(k,:)
            end
            trajectoryHandles = gobjects(N_a, 1); 
    
            p = x(1:2,:,:);
    
            for i = 1:N_a
                if N_a > 1
                    col = colors(i,:);
                else
                    col = colors(k,:);
                end
                trajectoryHandles(i) = plot(squeeze(p(1,i,:)), squeeze(p(2,i,:)), 'Color', col,...
                                        'LineWidth', 4,...
                                        'LineStyle', '-',...
                                        'DisplayName', 'temp');
                                        % 'DisplayName', [sprintf('Agent %d', i), '\hspace{1.2mm}']);
            end
            
            uistack(trajectoryHandles, 'bottom');
        end
    end
    
    % Save the figure if this function was run explicitly
    if nargin == 0
        timestamp = datestr(now, 'dd-mm_HH-MM');
        filename = ['Images/Simulations/static_trajectory_' timestamp '.fig'];
        saveas(gcf, filename);
    end
end


