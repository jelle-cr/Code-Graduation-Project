function plot_real_time_trajectories(x, y, Potential, Force, range, localMinX, localMinY, globalMinX, globalMinY, p, t, t_stop, delay)
    load('./Data/Parameters.mat')
    colors = lines(N_a);  

    % Animated Surface plot
    figure('Position', [100 50 800 700]);  %Left Bottom Width Height
    surf(x,y,Potential','FaceAlpha',1, 'EdgeColor','none')
    ax = gca;
    set(ax, 'FontSize', 12);
    xlim([-3 3]); ylim([-3 3]); zlim([0 4]);
    xlabel('$x$ [m]', 'Interpreter','latex', 'FontSize', 16);
    ylabel('$y$ [m]', 'Interpreter','latex', 'FontSize', 16);
    zlabel('$U(\mathbf{p}_{id},\mathbf{p}_{ij})$', 'Interpreter','latex', 'FontSize', 16);
    title('Total Potential Function', 'Interpreter', 'latex', 'FontSize', 18);
    hold on;
    % caxis([0, max(max(Potential))]);
    % 
    % % Step 2: Generate the sphere
    %     [r, s, z] = sphere;  % Creates sphere data (21x21 grid)
    % 
    %     % Step 3: Adjust the position and size of the sphere
    %     x_center = -2;  % X position of the sphere
    %     y_center = -2;  % Y position of the sphere
    %     z_center = 1; % Z position of the sphere
    %     radius = r_a;    % Radius of the sphere
    % 
    %     % Scale and position the sphere
    %     x_sphere = radius * r + p(1,1,1);
    %     y_sphere = radius * s + p(2,1,1);
    %     z_sphere = 0.8*radius * z + 1*radius + 1/2*K_att*norm(p(:,1,1) - p_d);
    % 
    %     % Step 4: Plot the sphere
    %     h_sphere = surf(x_sphere, y_sphere, z_sphere, 'FaceColor', 		"#0072BD", 'FaceAlpha', 1, 'EdgeColor', "black");  % Red sphere without edges
    % pause(delay)
    % for t = 1:1:(t_stop/t_step + 1)
    %     % Update the sphere's position data
    %     set(h_sphere, 'XData', radius * r + p(1,1,t));
    %     set(h_sphere, 'YData', radius * s + p(2,1,t));
    %     set(h_sphere, 'ZData', 0.8*radius * z + 1*radius +1/2*K_att*norm(p(:,1,t) - p_d));
    %     drawnow;
    % end

    % Surface plot
    figure('Position', [100 50 800 700]);  %Left Bottom Width Height
    surf(x,y,Potential','FaceAlpha',1, 'EdgeColor','none')
    ax = gca;
    set(ax, 'FontSize', 12);
    xlim([-3 3]); ylim([-3 3]); zlim([0 4]);
    xlabel('$x$ [m]', 'Interpreter','latex', 'FontSize', 16);
    ylabel('$y$ [m]', 'Interpreter','latex', 'FontSize', 16);
    zlabel('$U(\mathbf{p}_{id},\mathbf{p}_{ij})$', 'Interpreter','latex', 'FontSize', 16);
    title('Total Potential Function', 'Interpreter', 'latex', 'FontSize', 18);
    % hold on;

    % Contour/quiver plot
    figure('Position', [100 50 800 700]);  %Left Bottom Width Height
    hold on

    pause(delay);
    % step = floor(n/30);                % Subsample the amount of quiver vectors
    % quiver(x(1:step:end, 1:step:end), y(1:step:end, 1:step:end), Force(1:step:end, 1:step:end, 1)', Force(1:step:end, 1:step:end, 2)', 0.8, 'HandleVisibility', 'off');

    % Plot contour lines
    contour(x,y,Potential', 20, 'HandleVisibility', 'off');
    c = colorbar;

    % Plot obstacles
    plot(NaN, NaN, 'o', 'MarkerEdgeColor', 'black', ...
                        'MarkerFaceColor', 'black', ...
                        'MarkerSize', 15, ...
                        'DisplayName', 'Obstacle');
    for o = 1:N_o
        th = 0:pi/50:2*pi;
        x_circle = r_o * cos(th) + p_o(1,o);
        y_circle = r_o * sin(th) + p_o(2,o);
        patch(x_circle, y_circle, 'black', 'HandleVisibility', 'off');
    end

    % Formatting
    ax = gca;
    set(ax, 'FontSize', 12);
    xlabel('$x$ [m]', 'Interpreter','latex', 'FontSize', 16);
    ylabel('$y$ [m]', 'Interpreter','latex', 'FontSize', 16);
    ylabel(c, '$U(\mathbf{p}_{id})$', 'Interpreter','latex', 'FontSize', 16);
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
        % for i = 1:N_a    % This for loop is required to always plot the repulsion regions under the agents
        %     th = 0:pi/50:2*pi; % Angles for creating the circle shape
        %     x_circle = (r_a+rho_0) * cos(th) + p(1,i,t);
        %     y_circle = (r_a+rho_0) * sin(th) + p(2,i,t);
        %     if handles_repulsive(i) == 0
        %         handles_repulsive(i) = patch(x_circle, y_circle, colors(i,:), ...
        %                                       'FaceColor', colors(i,:), ...
        %                                       'FaceAlpha', 0.2, ...
        %                                       'EdgeColor', 'none', ...
        %                                       'HandleVisibility', 'off');
        %     else
        %         set(handles_repulsive(i), 'XData', x_circle, ...
        %                                'YData', y_circle);
        %     end
        % end
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
                                              'FaceAlpha', 0.4, ...
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
            % % Plot Local Minima
            % for i = 1:length(localMinX)
            %     if abs(localMinX(i) - globalMinX) > 1e-6 || abs(localMinY(i) - globalMinY) > 1e-6      % Exclude the Global Minimum
            %         plot(localMinX(i), localMinY(i), '*', 'lineWidth', 1.5, ...
            %                                  'MarkerSize', 12, ...
            %                                  'MarkerEdgeColor', 'green', ...
            %                                  'MarkerFaceColor', 'green', ...
            %                                  'HandleVisibility', 'off');
            %     else
            %         plot(NaN, NaN, '*', 'lineWidth', 1.5, ...
            %                             'MarkerEdgeColor', 'green', ...
            %                             'MarkerFaceColor', 'green', ...
            %             	            'MarkerSize', 12, ...
            %                             'DisplayName', 'Equilibrium Point');
            %     end
            % end

            % % Plot Global minimum
            % plot(globalMinX, globalMinY, '*', 'lineWidth', 1.5, ...
            %                                   'MarkerSize', 12, ...
            %                                   'MarkerEdgeColor', '#EDB120', ...
            %                                   'MarkerFaceColor', '#EDB120', ...
            %                                   'DisplayName', 'Global Minimum');

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
        title(['Real-time trajectory ($t$ = ', sprintf('%.2f', t*t_step-t_step), ' [s])'], ...
          'Interpreter', 'latex', 'FontSize', 18);

        % % Generate the sphere data
        % xpos = p(1,1,t);
        % xindex = find(abs(x-xpos)==min(abs(x-xpos)));
        % ypos = p(2,1,t);
        % yindex = find(abs(y-ypos)==min(abs(y-ypos)));
        % % [xpos, ypos, Potential(xindex, yindex)] = sphere;
        % 
        % [rX, rY, rZ] = sphere;
        % 
        % % Scale and position the sphere
        % r = 0.5;  % Radius of the sphere
        % x_center = 0;  % X-coordinate of the sphere's center
        % y_center = 0;  % Y-coordinate of the sphere's center
        % z_center = 1;  % Z-coordinate of the sphere's center (height)
        % 
        % % Plot the sphere with a single color (e.g., red)
        % sphereSurface = surf(xpos + r_a * rX, ypos + r_a * rY, Potential(xindex,yindex) + r_a * rZ);
        % set(sphereSurface, 'FaceColor', 'r', 'EdgeColor', 'none');  % Red sphere, no edges

        drawnow %limitrate; % Force plot update
    end
end