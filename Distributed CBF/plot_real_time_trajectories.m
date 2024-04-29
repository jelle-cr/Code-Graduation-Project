function plot_real_time_trajectories(p, t_step, N_a, update_interval, xlim_values, ylim_values, fontsize, agent_radius, linewidth, p_nom, u_nom, u, num_steps, t_span, t_stop)
    % PLOT_REAL_TIME_TRAJECTORIES Plots multiple trajectories in real time
    %   p: 3D matrix containing trajectories (x, y, time)
    %   N_a: Number of agents
    %   update_interval: Interval between updates in seconds (optional, default 0.1), inaccurate due to calculation time between each step
    
    % Colors for different trajectories
    colors = lines(N_a);  
    
    % Initialize subplots
    figure('Position', [100 150 1400 600]);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Subplot 1 Init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(1,2,1);
    grid on; hold on;
    axis equal;
    xlim(xlim_values); ylim(ylim_values);
    legend('Location', 'northeast', 'Interpreter', 'latex', 'FontSize', fontsize);
    title('Real-Time Trajectories', 'Interpreter', 'latex', 'FontSize', fontsize);
    xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', fontsize);
    ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', fontsize);
    
    % Storage for plot handles (markers and trails)
    circle_handles_nom = zeros(N_a, 1);
    trail_handles_nom = zeros(N_a, 1);
    circle_handles = zeros(N_a, 1);
    trail_handles = zeros(N_a, 1);
    
    % Collision detection
    overlap_marker = zeros(N_a, N_a);                % Initialize
    collision_occurred = false;         % Flag to add collision to legens
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Subplot 2 Init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(1,2,2);
    grid on; hold on;
    for agent_id = 1:N_a
        diff = squeeze(u_nom(:,agent_id,:))-squeeze(u(:,agent_id,:)); 
        for t = 1:num_steps
            norms(agent_id, t) = norm(diff(:, t)); % Calculate norm at each time step
        end
        plot(t_span, norms(agent_id, :),'LineWidth',1,'DisplayName', sprintf('Agent %d', agent_id));
    end
    xlim([0 t_span(end)]);
    legend('Location', 'northeast', 'Interpreter', 'latex', 'FontSize', fontsize);
    title('Norm $u_i - u_{0i}$ over time', 'Interpreter', 'latex', 'FontSize', fontsize);
    xlabel('$t$ [s]', 'Interpreter', 'latex', 'FontSize', fontsize);
    ylabel('$||u_i-u_{0i}||$', 'Interpreter', 'latex', 'FontSize', fontsize);
    
    % Storage for plot handles (markers)
    time_marker_norms = zeros(N_a,1);
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Main loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for t = 1:(t_stop+t_step + 0.00001)/t_step
        subplot(1,2,1);     % Update subplot 1
        for agent_id = 1:N_a
    
            % Update or create the nominal trail
            if trail_handles_nom(agent_id) == 0
                trail_handles_nom(agent_id) = plot(p_nom(1, agent_id, 1:t), p_nom(2, agent_id, 1:t), ...
                                               '.', 'Color', colors(agent_id,:), ...
                                               'LineWidth', linewidth, ...
                                               'HandleVisibility', 'off');
            else
                trail_time = 0.5;
                if(t*t_step > trail_time)
                    trail_start = t-(trail_time/t_step);
                else
                    trail_start = 1;
                end
                set(trail_handles_nom(agent_id), 'XData', p_nom(1, agent_id, trail_start:t), ...
                                             'YData', p_nom(2, agent_id, trail_start:t));
            end

            % Update or create the actual trail
            if trail_handles(agent_id) == 0
                trail_handles(agent_id) = plot(p(1, agent_id, 1:t), p(2, agent_id, 1:t), ...
                                               '-', 'Color', colors(agent_id,:), ...
                                               'LineWidth', linewidth, ...
                                               'HandleVisibility', 'off');
            else
                trail_time = 0.5;
                if(t*t_step > trail_time)
                    trail_start = t-(trail_time/t_step);
                else
                    trail_start = 1;
                end
                set(trail_handles(agent_id), 'XData', p(1, agent_id, trail_start:t), ...
                                             'YData', p(2, agent_id, trail_start:t));
            end

            x_current = p_nom(1, agent_id, t);
            y_current = p_nom(2, agent_id, t);

            % Update or create the nominal agent circle
            if circle_handles_nom(agent_id) == 0
                th = 0:pi/50:2*pi; % Angles for creating the circle shape
                xunit = agent_radius * cos(th) + x_current;
                yunit = agent_radius * sin(th) + y_current;
                circle_handles_nom(agent_id) = patch(xunit, yunit, colors(agent_id,:), ...
                                              'FaceColor', 'none', ...
                                              'EdgeColor', colors(agent_id,:), ...
                                              'HandleVisibility', 'off');
                % % Create circle in legend
                % plot(NaN, NaN, 'o', 'MarkerEdgeColor', colors(agent_id,:), ...
                %                               'MarkerFaceColor', colors(agent_id,:), ...
                %                               'MarkerSize', 15, ...
                %                               'DisplayName', sprintf('Agent %d', agent_id));
            else
                set(circle_handles_nom(agent_id), 'XData', agent_radius * cos(th) + x_current, ...
                                              'YData', agent_radius * sin(th) + y_current);
            end
    
            x_current = p(1, agent_id, t);
            y_current = p(2, agent_id, t);
            % Update or create the actual agent circle
            if circle_handles(agent_id) == 0
                th = 0:pi/50:2*pi; % Angles for creating the circle shape
                xunit = agent_radius * cos(th) + x_current;
                yunit = agent_radius * sin(th) + y_current;
                circle_handles(agent_id) = patch(xunit, yunit, colors(agent_id,:), ...
                                              'FaceColor', colors(agent_id,:), ...
                                              'EdgeColor', 'none', ...
                                              'HandleVisibility', 'off');
                % Create circle in legend
                plot(NaN, NaN, 'o', 'MarkerEdgeColor', colors(agent_id,:), ...
                                              'MarkerFaceColor', colors(agent_id,:), ...
                                              'MarkerSize', 15, ...
                                              'DisplayName', sprintf('Agent %d', agent_id));
            else
                set(circle_handles(agent_id), 'XData', agent_radius * cos(th) + x_current, ...
                                              'YData', agent_radius * sin(th) + y_current);
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

                    dx = p(1, i, t) - p(1, j, t);
                    dy = p(2, i, t) - p(2, j, t);
                    distance = sqrt(dx^2 + dy^2);
                    if distance < 2*agent_radius 
                        overlap_center_x = (p(1, i, t) + p(1, j, t))/2; 
                        overlap_center_y = (p(2, i, t) + p(2, j, t))/2; 

                        % Create new overlap marker (to show the collision)
                        overlap_marker(i,j) = plot(overlap_center_x, overlap_center_y, '*', ...
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
        title(['Real-Time Trajectories (t = ', sprintf('%.2f', t*t_step-t_step), '[s])'], ...
          'Interpreter', 'latex', 'FontSize', fontsize);
    
        subplot(1,2,2);     % Update subplot 2
        for agent_id = 1:N_a
            if time_marker_norms(agent_id) == 0
                time_marker_norms(agent_id) = plot(t*t_step, norms(agent_id,t), 'o', ...
                                                'Color', colors(agent_id,:), ...
                                                'MarkerSize', 10, ...
                                                'LineWidth', 2, ...
                                                'HandleVisibility', 'off');  % Initial placement
            else
                set(time_marker_norms(agent_id), 'XData', t*t_step, 'YData', norms(agent_id,t));
            end
        end
    
        drawnow %limitrate; % Force plot update
        pause(update_interval);
    end
end