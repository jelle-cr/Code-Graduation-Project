function plot_real_time_trajectories(X, t, r_a, rho_0)
    % PLOT_REAL_TIME_TRAJECTORIES Plots multiple trajectories in real time
    %   X: 3D matrix containing trajectories (x, y, time)
    %   N_a: Number of agents
    %   update_interval: Interval between updates in seconds (optional, default 0.1), inaccurate due to calculation time between each step

    N_a = size(X,2);
    n = size(X,1);

    fontsize = 16;

    % Colors for different trajectories
    colors = lines(N_a);  
    
    % Initialize subplots
    left = 500;
    bottom = 50;
    width = 800;
    height = 700;
    figure('Position', [left bottom width height]);  

    % pause(10);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Trajectory Plot Init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    grid on; hold on;
    plot(squeeze(X(1,:,:)).',squeeze(X(2,:,:)).', 'LineWidth',2, 'HandleVisibility', 'off'); % Plot the complete trajectories of all agents
    
    % The axis limits are calculated such that the resulting plot is
    % square and centred around the trajectories limits
    axis equal; % The x and y axis scaling should be equal
    xmin = min(min(X(1,:,:))); xmax = max(max(X(1,:,:)));
    ymin = min(min(X(2,:,:))); ymax = max(max(X(2,:,:)));
    xrange = xmax-xmin; yrange = ymax-ymin;
    if xrange>yrange
        axrange = xrange;
    else
        axrange = yrange;
    end
    xmin = xmin - (axrange-xrange)/2 - 2*r_a; xmax = xmax + (axrange-xrange)/2 + 2*r_a;
    ymin = ymin - (axrange-yrange)/2 - 2*r_a; ymax = ymax + (axrange-yrange)/2 + 2*r_a;
    xlim([xmin xmax]); ylim([ymin ymax]);

    % Generate the legend entries
    % legendEntries = cell(1, N_a);
    % for i = 1:N_a
    %     legendEntries{i} = ['Agent ' num2str(i)];
    % end

    ax = gca;
    set(ax, 'FontSize', fontsize-5);
    legend('Location', 'northeast', 'BackgroundAlpha', 0.3, 'Interpreter', 'latex', 'FontSize', fontsize);
    % title('Real-Time Trajectories', 'Interpreter', 'latex', 'FontSize', fontsize);
    xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', fontsize);
    ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', fontsize);
    

    % Storage for plot handles (markers and trails)
    circle_handles = zeros(N_a, 1);
    circle_handles_rep = zeros(N_a, 1);
 
    % Collision detection
    overlap_marker = zeros(N_a, N_a);   % Initialize
    collision_occurred = false;         % Flag to add collision to legend


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Main loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num_steps = length(t);
    t_step = t(2)-t(1);
    for t = 1:num_steps
        % plot();     % Update subplot 1
        for agent_id = 1:N_a    % This for loop is required to always plot the repulsion regions under the agents
            x_current = X(1,agent_id,t);
            y_current = X(2,agent_id,t);
            % Update or create the actual agent circle
            if circle_handles_rep(agent_id) == 0
                th = 0:pi/50:2*pi; % Angles for creating the circle shape
                xunit = (r_a+rho_0) * cos(th) + x_current;
                yunit = (r_a+rho_0) * sin(th) + y_current;
                circle_handles_rep(agent_id) = patch(xunit, yunit, colors(agent_id,:), ...
                                              'FaceColor', colors(agent_id,:), ...
                                              'FaceAlpha', 0.2, ...
                                              'EdgeColor', 'none', ...
                                              'HandleVisibility', 'off');
            else
                set(circle_handles_rep(agent_id), 'XData', (r_a+rho_0) * cos(th) + x_current, ...
                                              'YData', (r_a+rho_0) * sin(th) + y_current);
            end
        end
        for agent_id = 1:N_a    % This for loop is required to always plot the agents on top of the trajectory
            x_current = X(1,agent_id,t);
            y_current = X(2,agent_id,t);
            % Update or create the actual agent circle
            if circle_handles(agent_id) == 0
                th = 0:pi/50:2*pi; % Angles for creating the circle shape
                xunit = r_a * cos(th) + x_current;
                yunit = r_a * sin(th) + y_current;
                circle_handles(agent_id) = patch(xunit, yunit, colors(agent_id,:), ...
                                              'FaceColor', colors(agent_id,:), ...
                                              'FaceAlpha', 0.9, ...
                                              'EdgeColor', 'none', ...
                                              'HandleVisibility', 'off');
                % Create circle in legend
                plot(NaN, NaN, 'o', 'MarkerEdgeColor', colors(agent_id,:), ...
                                              'MarkerFaceColor', colors(agent_id,:), ...
                                              'MarkerSize', 15, ...
                                              'DisplayName', [sprintf('Agent %d', agent_id), '\hspace{1.2mm}']);
            else
                set(circle_handles(agent_id), 'XData', r_a * cos(th) + x_current, ...
                                              'YData', r_a * sin(th) + y_current);
            end
        end
        % Collision Detection and plotting
        for i = 1:N_a
            for j = 1:N_a
                if i ~= j
                    % Remove previous overlap marker (if any)
                    if ~isequal(overlap_marker(i,j),0)  % No clue why, but this works :)
                        delete(overlap_marker(i,j));
                        overlap_marker(i,j) = 0;
                    end
                    
                    dist = X(:,i,t) - X(:,j,t);
                    if dist.'*dist < (2*r_a)^2 
                        overlap_center = abs(X(:,i,t)+X(:,j,t))/2; 
                        
                        % Create new overlap marker (to show the collision)
                        overlap_marker(i,j) = plot(overlap_center(1), overlap_center(2), '*', ...
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
        % Update time in title
        title(['Real-Time Trajectories ($t$ = ', sprintf('%.2f', t*t_step-t_step), ' [s])'], ...
          'Interpreter', 'latex', 'FontSize', fontsize);

        drawnow %limitrate; % Force plot update
    end    
end