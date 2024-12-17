function plot_over_time(data, t_step, t_end, label, save)
    
    if nargin == 0
        close all
        load('./Data/SimulationDataRecent.mat', 'u_att', 'u_rep', 't_step', 't_end');
        data = vecnorm(squeeze(u_att(:,1,:)) + squeeze(u_rep(:,1,:)));
        label = 'temp';
        save = false;
    end
    t = 0:t_step:t_end;

    load('+Functions\customColors.mat');
    colors = DesmosColors;

    %% Setup plot
    figure('Position', [100 250 1000 500]);  %Left Bottom Width Height
    hold on; grid on;
    ax = gca; set(ax, 'FontSize', 22); ax.TickLabelInterpreter = 'latex';
    xlim([t(1), t(end)]); %ylim(rangeY);
    xticks(t(1):0.5:t(end));
    xlabel('$t$ [s]', 'Interpreter','latex', 'FontSize', 30);
    ylabel(['$' label '$'], 'Interpreter','latex', 'FontSize', 30);

    numPlots = height(data);
    for i = 1:numPlots
        plot(t, data(i,:), 'Color', colors(i,:), 'LineWidth', 4);
    end

    legend('Location', 'northeast', ...
           'Interpreter', 'latex', ...
           'FontSize', 26,...
           'BackgroundAlpha',0.6);

    if save
        timestamp = datestr(now, 'dd-mm_HH-MM-SS');
        filename = ['Images/Simulations/plot_over_time_' timestamp '.fig'];
        saveas(gcf, filename);
        pause(1)
    end
end