function plot_trajectory(rangeX, rangeY, x, x_0, x_d, x_o, r_o, N_o)
    p = x(1:2,:);
    p_0 = x_0(1:2);
    p_d = x_d(1:2);
    p_o = x_o(1:2,:);

    %% Setup plot
    figure('Position', [100 50 700 450]);  %Left Bottom Width Height
    hold on; grid on; axis equal; 
    ax = gca; set(ax, 'FontSize', 18); ax.TickLabelInterpreter = 'latex';
    xlim(rangeX); ylim(rangeY);
    xticks(rangeX(1):1:rangeX(2)); yticks(rangeY(1):1:rangeY(2));

    % Plot obstacles
    th = 0:pi/50:2*pi;
    for j = 1:N_o
        x_obs = r_o * cos(th) + p_o(1,j);
        y_obs = r_o * sin(th) + p_o(2,j);
        patch(x_obs, y_obs, 'black', 'FaceColor', '#aaaaaa', ...
                                     'FaceAlpha', 1,...
                                     'EdgeColor', 'black', ...
                                     'HandleVisibility', 'off');
    end

    % Plot initial position
    plot(p_0(1), p_0(2), 'o','MarkerSize', 20, ...
                             'MarkerEdgeColor', 'blue', ...
                             'LineWidth', 4,...
                             'HandleVisibility', 'off');

    % Plot desired position
    plot(p_d(1), p_d(2), 'x','MarkerSize', 30, ...
                             'MarkerEdgeColor', 'red', ...
                             'LineWidth', 6,...
                             'HandleVisibility', 'off');

    %% Trajectory plotting
    plot(p(1,:), p(2,:), 'LineWidth', 2, 'Color', 'blue');
end


