function plot_static_trajectories(rangeX, rangeY, plottingFolder)

    if nargin == 0
        close all
        rangeX = [-3; 3];
        rangeY = [-2; 2];
        % Place a Parameter file, along with all of the simulation outputs in the plottingFolder
        plottingFolder = 'Data/plottingData';
    end

    % Get a list of all .mat files in the plottingFolder
    fileList = dir(fullfile(plottingFolder, '*.mat'));

    % Select first file and load specific parameters
    params = fullfile(fileList(1).folder, fileList(1).name);
    load(params, 'x_0', 'x_d', 'x_o', 'N_a', 'r_o', 'N_o', 't_step', 't_end');
   
    quality = 'SD'; % Standard Definition
    [~] = Functions.trajectory_plot_setup(rangeX, rangeY, x_0, x_d, x_o, N_a, r_o, N_o, quality);      % Sets up obstacles, initial and desired positions, and legend
    
    
    %% Trajectory plotting
    load('+Functions\customColors.mat');
    colors = DesmosColors;
    if ~isempty(fileList)
        for k = 1:length(fileList)   % Reverse order, to plot last file on top
            % Load desired datasets
            dataFile = fullfile(fileList(k).folder, fileList(k).name);
            fprintf('Processing file: %s\n', dataFile);
            load(dataFile, 'x');           
           
            trajectoryHandles = gobjects(N_a, 1); 

            timepointInterval = 0.5;                  % Plots a point every 0.5 s
            timepointHandles = gobjects(N_a, 1); 
            t = 0:t_step:t_end;
            tp_ind = mod(t/0.5,1) == 0;
    
            p = x(1:2,:,:);
    
            for i = 1:N_a
                if N_a > 1
                    col = colors(i,:);
                else
                    col = colors(k,:);
                end
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
        end
    end
    
    % Save the figure if this function was run explicitly
    if nargin == 0
        timestamp = datestr(now, 'dd-mm_HH-MM-SS');
        filename = ['Images/Simulations/static_trajectory_' timestamp '.fig'];
        saveas(gcf, filename);
    end
end


