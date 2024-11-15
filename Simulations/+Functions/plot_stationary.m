function plot_stationary(range, t_stop)
    load('Data/Parameters.mat');
    load('Data/SimulationData.mat');
    t = 0:t_step:t_end; 
    colors = lines(N_a);  
    p_d = x_d(1:2);
    p_o = x_o(1:2);

    p = x(1:2,:,:);


    %% Setup plot
    figure('Position', [100 50 800 700]);  %Left Bottom Width Height
    hold on; grid on; axis equal; 
    ax = gca; set(ax, 'FontSize', 12);
    xlim(range); ylim(range);
    xlabel('$x_1$ [m]', 'Interpreter','latex', 'FontSize', 16);
    ylabel('$x_2$ [m]', 'Interpreter','latex', 'FontSize', 16);

    %% Create legend entries
    legend('Location', 'northwest', 'BackgroundAlpha', 0.7, 'Interpreter', 'latex', 'FontSize', 14);
    % Create goal in legend
    plot(NaN, NaN, 'x', 'MarkerSize', 12, ...
                        'MarkerEdgeColor', 'black', ...
                        'LineWidth', 2,...
                        'DisplayName', 'Desired Position');
    % Create initial positions in legend
    plot(NaN, NaN, 'o', 'MarkerSize', 10, ...
                        'MarkerEdgeColor', 'black', ...
                        'LineWidth', 2, ...
                        'DisplayName', 'Initial Positions');
    % Create final positions in legend
    plot(NaN, NaN, '^', 'MarkerSize', 10, ...
                        'MarkerEdgeColor', 'black', ...
                        'LineWidth', 2, ...
                        'DisplayName', 'Final Positions');

    %% Trajectory plotting
    for i = 1:N_a
        plot(squeeze(p(1,i,:)), squeeze(p(2,i,:)), 'Color', colors(i,:),...
                                                   'LineWidth', 2,...
                                                   'LineStyle', '-');
    end

    %% Static plotting
    % Plot initial positions
    for i = 1:N_a
        plot(squeeze(p(1,i,1)), squeeze(p(2,i,1)), 'o','MarkerSize', 10, ...
                                                       'MarkerEdgeColor', colors(i,:), ...
                                                       'LineWidth', 2,...
                                                       'HandleVisibility', 'off');
    end    
    % Plot final positions
    for i = 1:N_a
        plot(squeeze(p(1,i,end)), squeeze(p(2,i,end)), '^','MarkerSize', 10, ...
                                                           'MarkerEdgeColor', colors(i,:), ...
                                                           'LineWidth', 2,...
                                                           'HandleVisibility', 'off');
    end  
    % Plot Goal
    plot(p_d(1), p_d(2), 'x','MarkerSize', 20, ...
                             'MarkerEdgeColor', 'black', ...
                             'LineWidth', 4,...
                             'HandleVisibility', 'off');


    
end

