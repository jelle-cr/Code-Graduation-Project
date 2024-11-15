function plot_real_time_trajectories(range, p, t, t_stop, delay)
    load('./Data/Parameters.mat')
    p_0 = p_0(1:2,:);
    p_d = p_d(1:2,1);
    p_o = p_o(1:2,:);
    colors = lines(N_a);  

    % Trajectory plot
    figure('Position', [100 50 800 700]);  %Left Bottom Width Height
    hold on
    grid on

    if delay > 0
        pause(delay);
    end
    
    % Plot obstacles
    plot(NaN, NaN, 'o', 'MarkerEdgeColor', 'black', ...
                        'MarkerFaceColor', 'black', ...
                        'MarkerSize', 15, ...
                        'DisplayName', 'Obstacle');

    %% Obstacle
    poly = [-1, 1, 1.2, -0.5, -1.5;
            -2, -1, 1,  2, 0];
    for o = 1:N_o
        % th = 0:pi/50:2*pi;
        % x_circle = r_o * cos(th) + p_o(1,o);
        % y_circle = r_o * sin(th) + p_o(2,o);
        % patch(x_circle, y_circle, 'black', 'HandleVisibility', 'off');
        patch([poly(1,:), poly(1,1)], [poly(2,:), poly(2,1)], 'black', 'HandleVisibility', 'off'); 
    end
    
    % Formatting
    ax = gca;
    set(ax, 'FontSize', 12);
    xlabel('$x$ [m]', 'Interpreter','latex', 'FontSize', 16);
    ylabel('$y$ [m]', 'Interpreter','latex', 'FontSize', 16);
    legend('Location', 'northwest', 'BackgroundAlpha', 0.7, 'Interpreter', 'latex', 'FontSize', 14);
    axis equal
    xlim([-range, range]);
    ylim([-range, range]);

    % Real-time trajectories
    % Storage for plot handles (markers and trails)
    handles_agents = zeros(N_a, 1);         % Handles for agents
    handles_repulsive = zeros(N_a, 1);      % Handles for repulsive region
    handles_trajectories = zeros(N_a, 1);   % Handles for trajectories

    for t = 1:1:(t_stop/t_step + 1)
        for i = 1:N_a    % This for loop plots the trajectories
            % Update or create the trajectory
            if handles_trajectories(i) == 0
                handles_trajectories(i) = plot(p(1,i,1:t), p(2,i,1:t), ...
                                       'Color', colors(i,:), ...
                                       'LineWidth', 2, ...
                                       'LineStyle', '--', ...
                                       'HandleVisibility', 'off');
            else
                set(handles_trajectories(i), 'XData', p(1,i,1:t), ...
                                             'YData', p(2,i,1:t));
            end
        end
        for i = 1:N_a    % This for loop plots the agents 
            th = 0:pi/50:2*pi; % Angles for creating the circle shape
            x_circle = r_a * cos(th) + p(1,i,t);
            y_circle = r_a * sin(th) + p(2,i,t);
            % Update or create the actual agent circle
            if handles_agents(i) == 0
                handles_agents(i) = patch(x_circle, y_circle, colors(i,:), ...
                                              'FaceColor', colors(i,:), ...
                                              'FaceAlpha', 0.1, ...
                                              'EdgeColor', colors(i,:), ...
                                              'HandleVisibility', 'off');
                if i == 1 && t == 1
                    % Create circle in legend
                    plot(NaN, NaN, 'o', 'MarkerEdgeColor', colors(1,:), ...
                                        'MarkerFaceColor', colors(1,:), ...
                                        'MarkerSize', 15, ...
                                        'DisplayName', 'Agent');
                end
            else
                set(handles_agents(i), 'XData', x_circle, ...
                                       'YData', y_circle);
            end
        end
        if t == 1               % Plot these once, on top of agents      
            % Plot initial positions
            plot(NaN, NaN, 'o','MarkerSize', 10, ...
                               'MarkerEdgeColor', 'blue', ...
                               'LineWidth', 2,...
                               'DisplayName', 'Initial Position');
            for i = 1:N_a
                plot(p_0(1,i), p_0(2,i), 'o','MarkerSize', 10, ...
                               'MarkerEdgeColor', colors(i,:), ...
                               'LineWidth', 2,...
                               'HandleVisibility', 'off');
            end
            
            % Plot Goal
            plot(p_d(1), p_d(2), 'x','MarkerSize', 12, ...
                                     'MarkerEdgeColor', 'red', ...
                                     'LineWidth', 2,...
                                     'DisplayName', 'Desired Position');
        end

        % Update time in title
        title(['Trajectory around Polytope ($t$ = ', sprintf('%.2f', t*t_step-t_step), ' [s])'], ...
          'Interpreter', 'latex', 'FontSize', 18);

        drawnow %limitrate; % Force plot update
    end
end