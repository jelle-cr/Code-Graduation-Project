function plot_real_time_trajectories(p, range, t_stop, t_step, delay)
    load('Data/Parameters.mat');    
    p_d = q_d(1:2);
    p_o = q_o(1:2);
    colors = lines(N_a);  

    %% Setup plot
    figure('Position', [100 50 800 700]);  %Left Bottom Width Height
    hold on; grid on; axis equal; 
    ax = gca; set(ax, 'FontSize', 12);
    xlim([-range range]); ylim([-range range]);
    xlabel('$x$ [m]', 'Interpreter','latex', 'FontSize', 16);
    ylabel('$y$ [m]', 'Interpreter','latex', 'FontSize', 16);
    legend('Location', 'northwest', 'BackgroundAlpha', 0.7, 'Interpreter', 'latex', 'FontSize', 14);
    title('Trajectory over time ($t$ = 0 [s])', ...
          'Interpreter', 'latex', 'FontSize', 18);

    % Create agent in legend
    plot(NaN, NaN, 'o', 'MarkerEdgeColor', colors(1,:), ...
                        'MarkerFaceColor', colors(1,:), ...
                        'MarkerSize', 15, ...
                        'DisplayName', 'Agent');

    if delay > 0
        pause(delay);
    end

    %% Plot obstacles
    plot(NaN, NaN, 'o', 'MarkerEdgeColor', 'black', ...
                        'MarkerFaceColor', 'black', ...
                        'MarkerSize', 15, ...
                        'DisplayName', 'Obstacle');
    th = 0:pi/50:2*pi;
    x_circle = r_o * cos(th) + p_o(1);
    y_circle = r_o * sin(th) + p_o(2);
    patch(x_circle, y_circle, 'black', 'HandleVisibility', 'off');

    %% Real-time trajectories
    % Storage for plot handles (markers and trails)
    handles_agents = zeros(N_a, 1);         % Handles for agents
    handles_trajectories = zeros(N_a, 1);   % Handles for trajectories

    for t_ind = 1:1:(t_stop/t_step + 1)
        % Update or create the trajectory
        if handles_trajectories == 0
            handles_trajectories = plot(p(1,1:t_ind), p(2,1:t_ind), ...
                                       'Color', colors(1,:), ...
                                       'LineWidth', 2, ...
                                       'LineStyle', '--', ...
                                       'HandleVisibility', 'off');
        else
            set(handles_trajectories, 'XData', p(1,1:t_ind), ...
                                      'YData', p(2,1:t_ind));
        end
        th = 0:pi/50:2*pi; % Angles for creating the circle shape
        x_circle = r_a * cos(th) + p(1,t_ind);
        y_circle = r_a * sin(th) + p(2,t_ind);
        % Update or create the actual agent circle
        if handles_agents == 0
            handles_agents = patch(x_circle, y_circle, colors(1,:), ...
                                              'FaceColor', colors(1,:), ...
                                              'FaceAlpha', 0.1, ...
                                              'EdgeColor', colors(1,:), ...
                                              'HandleVisibility', 'off');
        else
            set(handles_agents, 'XData', x_circle, ...
                                'YData', y_circle);
        end
        if t_ind == 1               % Plot these once, on top of agents
            % Plot initial positions
            plot(NaN, NaN, 'o','MarkerSize', 10, ...
                               'MarkerEdgeColor', 'blue', ...
                               'LineWidth', 2,...
                               'DisplayName', 'Initial Positions');
            for i = 1:N_a
                plot(q_0(1,i), q_0(2,i), 'o','MarkerSize', 10, ...
                               'MarkerEdgeColor', colors(i,:), ...
                               'LineWidth', 2,...
                               'HandleVisibility', 'off');
            end
            
            % Plot Goal
            plot(q_d(1), q_d(2), 'x','MarkerSize', 12, ...
                                     'MarkerEdgeColor', 'red', ...
                                     'LineWidth', 2,...
                                     'DisplayName', 'Desired Position');
        end

        % Update time in title
        title(['Trajectories From Various Initial Positions ($t$ = ', sprintf('%.2f', t_ind*t_step-t_step), ' [s])'], ...
          'Interpreter', 'latex', 'FontSize', 18);

        drawnow %limitrate; % Force plot update
    end
end