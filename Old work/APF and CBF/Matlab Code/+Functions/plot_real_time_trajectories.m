function plot_real_time_trajectories(p, t, t_stop)
    % PLOT_REAL_TIME_TRAJECTORIES Plots multiple trajectories in real time
    %   p: 3D matrix containing trajectories ([x; y], agent_i, time)

    load('./Data/Parameters.mat')

    fontsize = 16;

    % Colors for different trajectories
    colors = lines(N_a);  
    
    % Initialize subplots
    left = 500;
    bottom = 50;
    width = 800;
    height = 700;
    figure('Position', [left bottom width height]);  

    % pause(5);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Trajectory Plot Init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    grid on; hold on;
    plot(squeeze(p(1,:,:)).',squeeze(p(2,:,:)).', 'LineWidth',2, 'HandleVisibility', 'off'); % Plot the complete trajectories of all agents
    for i = 1:N_a
        plot(squeeze(p(1,i,1)).', squeeze(p(2,i,1)).','o','MarkerSize', 8, ...
                                  'MarkerEdgeColor', colors(i,:), ...
                                  'LineWidth', 2,...
                                  'HandleVisibility', 'Off');
    end

    % The axis limits are calculated such that the resulting plot is
    % square and centred around the trajectories limits
    axis equal; % The x and y axis scaling should be equal
    xmin = min(min(p(1,:,:))); xmax = max(max(p(1,:,:)));
    ymin = min(min(p(2,:,:))); ymax = max(max(p(2,:,:)));
    xrange = xmax-xmin; yrange = ymax-ymin;
    if xrange>yrange
        axrange = xrange;
    else
        axrange = yrange;
    end
    xmin = xmin - (axrange-xrange)/2 - 2*r_a; xmax = xmax + (axrange-xrange)/2 + 2*r_a;
    ymin = ymin - (axrange-yrange)/2 - 2*r_a; ymax = ymax + (axrange-yrange)/2 + 2*r_a;
    xlim([xmin xmax]); ylim([ymin ymax]);

    ax = gca;
    set(ax, 'FontSize', fontsize-5);
    legend('Location', 'northwest', 'BackgroundAlpha', 0.3, 'Interpreter', 'latex', 'FontSize', fontsize);
    xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', fontsize);
    ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', fontsize);

    % Set Legend
    plot(NaN, NaN, 'o','MarkerSize', 10, ...
                       'MarkerEdgeColor', 'blue', ...
                       'LineWidth', 2,...
                       'DisplayName', 'Initial Positions');
    plot(NaN, NaN, 'x','MarkerSize', 12, ...
                       'MarkerEdgeColor', 'red', ...
                       'LineWidth', 2,...
                       'DisplayName', 'Desired Positions');

    % Storage for plot handles (markers and trails)
    handles_agents = zeros(N_a, 1);         % Handles for agents
    handles_repulsive = zeros(N_a, 1);      % Handles for repulsive region
    handles_goal = zeros(N_a, 1);           % Handles for desired positions

    % Collision detection 
    overlap_marker_agents = zeros(N_a, N_a);   % Initialize agents
    if N_o > 0
        overlap_marker_obstacles = zeros(N_a, N_o);   % Initialize obstacles
    end
    collision_occurred = false;         % Flag to add collision to legend
    
    % Plot obstacles
    if N_o > 0
        for o = 1:N_o
            if o == 1
                % Create circle in legend
                plot(NaN, NaN, 'o', 'MarkerEdgeColor', 'black', ...
                                              'MarkerFaceColor', 'black', ...
                                              'MarkerSize', 15, ...
                                              'DisplayName', 'Obstacle');
            end
            th = 0:pi/50:2*pi;
            x_circle = r_o * cos(th) + p_o(1,o);
            y_circle = r_o * sin(th) + p_o(2,o);
            patch(x_circle, y_circle, 'black', 'HandleVisibility', 'off');
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Main loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num_steps = length(t);
    t_step = t(2)-t(1);
    for t = 1:1:(t_stop/t_step + 1)
        tic
        for i = 1:N_a    % This for loop is required to always plot the repulsion regions under the agents
            th = 0:pi/50:2*pi; % Angles for creating the circle shape
            x_circle = (r_a+rho_0) * cos(th) + p(1,i,t);
            y_circle = (r_a+rho_0) * sin(th) + p(2,i,t);
            if handles_repulsive(i) == 0
                handles_repulsive(i) = patch(x_circle, y_circle, colors(i,:), ...
                                              'FaceColor', colors(i,:), ...
                                              'FaceAlpha', 0.2, ...
                                              'EdgeColor', 'none', ...
                                              'HandleVisibility', 'off');
            else
                set(handles_repulsive(i), 'XData', x_circle, ...
                                       'YData', y_circle);
            end
        end
        for i = 1:N_a    % This for loop is required to always plot the agents on top of the trajectory
            th = 0:pi/50:2*pi; % Angles for creating the circle shape
            x_circle = r_a * cos(th) + p(1,i,t);
            y_circle = r_a * sin(th) + p(2,i,t);
            % Update or create the actual agent circle
            if handles_agents(i) == 0
                handles_agents(i) = patch(x_circle, y_circle, colors(i,:), ...
                                              'FaceColor', colors(i,:), ...
                                              'FaceAlpha', 0.85, ...
                                              'EdgeColor', 'none', ...
                                              'HandleVisibility', 'off');
                % Create circle in legend
                plot(NaN, NaN, 'o', 'MarkerEdgeColor', colors(i,:), ...
                                    'MarkerFaceColor', colors(i,:), ...
                                    'MarkerSize', 15, ...
                                    'DisplayName', [sprintf('Agent %d', i), '\hspace{1.2mm}']);
            else
                set(handles_agents(i), 'XData', x_circle, ...
                                       'YData', y_circle);
            end
        end
        for i = 1:N_a    % This for loop is required to always plot the repulsion regions under the agents
            if handles_goal(i) == 0
                handles_goal(i) = plot(p_d(1,i,t), p_d(2,i,t), ...
                                      'x','MarkerSize', 10, ...
                                      'MarkerEdgeColor', colors(i,:), ...
                                      'LineWidth', 2,...
                                      'HandleVisibility', 'off');
            else
                set(handles_goal(i), 'XData', p_d(1,i,t), ...
                                     'YData', p_d(2,i,t));
            end
        end
        % Collision Detection for agents
        for i = 1:N_a
            for j = 1:N_a
                if i ~= j
                    % Remove previous overlap marker (if any)
                    if ~isequal(overlap_marker_agents(i,j),0)  % No clue why, but this works :)
                        delete(overlap_marker_agents(i,j));
                        overlap_marker_agents(i,j) = 0;
                    end

                    dist = p(1:2,i,t) - p(1:2,j,t);
                    if dist.'*dist < (2*r_a)^2 
                        overlap_center = (p(1:2,i,t)+p(1:2,j,t))/2; 

                        % Create new overlap marker (to show the collision)
                        overlap_marker_agents(i,j) = plot(overlap_center(1), overlap_center(2), '*', ...
                                                  'Color', 'red', 'MarkerSize', 14, ...
                                                  'HandleVisibility', 'off');
                        % On first collision add marker to legend
                        if ~collision_occurred  
                            plot(NaN, NaN, '*', 'Color', 'red', 'MarkerSize', 14, ...
                                                'DisplayName', sprintf('Collision'));
                            collision_occurred = true;  % Flag = true
                        end
                    end
                end
            end
        end
        % Collision Detection for obstacles
        if N_o > 0
            for i = 1:N_a
                for o = 1:N_o
                    % Remove previous overlap marker (if any)
                    if ~isequal(overlap_marker_obstacles(i,o),0)  % No clue why, but this works :)
                        delete(overlap_marker_obstacles(i,o));
                        overlap_marker_obstacles(i,o) = 0;
                    end

                    dist = p(1:2,i,t) - p_o(:,o);
                    if dist.'*dist < (r_a + r_o)^2 
                        overlap_center = (p(1:2,i,t)+p_o(:,o))/2; 

                        % Create new overlap marker (to show the collision)
                        overlap_marker_obstacles(i,o) = plot(overlap_center(1), overlap_center(2), '*', ...
                                                  'Color', 'magenta', 'MarkerSize', 14, ...
                                                  'HandleVisibility', 'off');
                        % On first collision add marker to legend
                        if ~collision_occurred  
                            plot(NaN, NaN, '*', 'Color', 'red', 'MarkerSize', 14, ...
                                                'DisplayName', sprintf('Collision'));
                            collision_occurred = true;  % Flag = true
                        end
                    end
                end
            end
        end
        % Update time in title
        title(['Real-Time Trajectories ($t$ = ', sprintf('%.2f', t*t_step-t_step), ' [s])'], ...
          'Interpreter', 'latex', 'FontSize', fontsize);

        drawnow %limitrate; % Force plot update
        toc
    end    
end