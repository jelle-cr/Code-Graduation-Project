close all
clear all


rangeX = [-3; 3];
rangeY = [-3; 3];
num_steps = 3000;
x = linspace(rangeX(1), rangeX(2), num_steps);
y = linspace(rangeY(1), rangeY(2), num_steps);

k_att = 1;
p_d = [-2; -2];


k_rep = 1;
r_a = 0.2;
N_o = 1;
p_o = [0; 0];
r_o = 0.5;
rho_0 = 10;


%% Calculate Potential Field
U_att = zeros(length(x), length(y));
U_rep = zeros(length(x), length(y));
for i = 1:length(x)
    for j = 1:length(y)
        p = [x(i); y(j)];
        U_att(i,j) = 1/2*k_att*norm(p - p_d)^2;

        for o = 1:N_o
            rho = norm(p - p_o(:,o)) - r_a - r_o;   
            if rho < rho_0
                rho = max(rho, 1e-6);           % In order to fill in the cylinder
                U_rep(i,j) = U_rep(i,j) + 1/2*k_rep*(1/rho - 1/rho_0)^2;
            end
        end
    end
end


Potential = U_att + U_rep;
Potential = min(Potential, 1*max(max(U_att)));


% figure('Position', [100 50 800 700]);  %Left Bottom Width Height 
figure('Position', [100, 50, 1200, 1600]);  %Left Bottom Width Height 
hold on; grid on;

surf(x,y,Potential','FaceAlpha',1, 'EdgeColor','none')

% set(gcf, 'Renderer', 'painters');
% set(gcf,'rendererMode','manual');

ax = gca; set(ax, 'FontSize', 20); ax.TickLabelInterpreter = 'latex';
zlimits = [0, max(max(Potential))];
xlim([-3 3]); ylim([-3 3]); zlim(zlimits);
% xlabel('$x_1$', 'Interpreter','latex', 'FontSize', 16); ylabel('$x_2$', 'Interpreter','latex', 'FontSize', 16);
% zlabel('$U_{\textnormal{att}}(\mathbf{x})$', 'Interpreter','latex', 'FontSize', 16);
% zlabel('$U_{\textnormal{rep}}(\mathbf{x})$', 'Interpreter','latex', 'FontSize', 16);
% zlabel('$U_{{\scriptscriptstyle \!\textnormal{APF}}}(\mathbf{x})$', 'Interpreter','latex', 'FontSize', 16);
ax.XTick = [-3, -2, -1, 0, 1, 2, 3]; % Set tick positions
% ax.XTickLabel = {'', '', '', '$x_1$', '', '', '', 'Interpreter', 'latex'}; % Set custom tick labels
ax.XTickLabel = {'', '', '', '', '', '', '', 'Interpreter', 'latex'}; % Set custom tick labels
ax.YTick = [-3, -2, -1, 0, 1, 2, 3]; % Set tick positions
% ax.YTickLabel = {'', '', '', '$x_2$', '', '', '', 'Interpreter', 'latex'}; % Set custom tick labels
ax.YTickLabel = {'', '', '', '', '', '', '', 'Interpreter', 'latex'}; % Set custom tick labels
ax.ZTick = [0, 5, 10, 15, 20, 25]; % Set tick positions
ax.ZTickLabel = {'', '', '', '', '', '', 'Interpreter', 'latex'}; % Set custom tick labels

% title('Total Potential Function', 'Interpreter', 'latex', 'FontSize', 18);

colormap parula;
clim(zlimits);
view(-25,45)

% shading interp;

% Save the figure as an PNG file
saveas(gcf, 'Images/potential_field.png');

% Save the figure as an SVG file
% saveas(gcf, 'Images/potential_field.svg');
% exportgraphics(gcf, 'Images/potential_field.pdf');

% Set up lighting
% light('Position', [2.5 0 20], 'Style', 'infinite'); % Add light source
% lighting phong; % Use smooth lighting model
% % material shiny; % Make surface material reflective and shiny
% 
% % Add shadow by adjusting the colormap and view
% shading interp; % Interpolate colors between grid points
% camlight left; % Add a secondary light for depth


% % Hill Terrain Simulation Script
% 
% % Define grid
% x = -10:0.1:10;
% y = -10:0.1:10;
% [X, Y] = meshgrid(x, y);
% 
% % Define terrain surface as a hill-type function
% Z = exp(-0.1 * (X.^2 + Y.^2)) - 0.5 * exp(-0.05 * ((X - 5).^2 + (Y + 5).^2));
% 
% % Plot terrain
% figure;
% surf(X, Y, Z, 'EdgeColor', 'none');
% colormap('parula'); % You can change the colormap here
% colorbar;
% hold on;
% 
% % Add lighting
% light('Position', [10 10 10], 'Style', 'infinite');
% lighting gouraud;
% 
% % Define goal position and obstacles
% goal = [-7, -7]; % Goal coordinates
% obstacles = [3, 3; 7, -5]; % Each row represents an obstacle [x, y]
% 
% % Plot goal and obstacles
% plot3(goal(1), goal(2), interp2(X, Y, Z, goal(1), goal(2)), 'ro', ...
%     'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Goal marker
% for i = 1:size(obstacles, 1)
%     plot3(obstacles(i, 1), obstacles(i, 2), ...
%         interp2(X, Y, Z, obstacles(i, 1), obstacles(i, 2)), 'ko', ...
%         'MarkerSize', 10, 'MarkerFaceColor', 'k'); % Obstacle marker
% end
% 
% % Agent initialization
% agent_pos = [8, 8]; % Starting position
% agent_marker = plot3(agent_pos(1), agent_pos(2), ...
%     interp2(X, Y, Z, agent_pos(1), agent_pos(2)), 'bo', ...
%     'MarkerSize', 8, 'MarkerFaceColor', 'b'); % Agent marker
% 
% % Simulation parameters
% max_steps = 200;
% learning_rate = 0.1; % Descent step size
% 
% % Simulation loop
% for step = 1:max_steps
%     % Gradient descent to update agent position
%     grad_x = -interp2(X, Y, gradient(Z, x, y), agent_pos(1), agent_pos(2), 'linear', 1);
%     grad_y = -interp2(X, Y, gradient(Z, y, x), agent_pos(1), agent_pos(2), 'linear', 1);
% 
%     % Move agent in the direction of the gradient
%     agent_pos(1) = agent_pos(1) + learning_rate * grad_x;
%     agent_pos(2) = agent_pos(2) + learning_rate * grad_y;
% 
%     % Update agent marker position
%     set(agent_marker, 'XData', agent_pos(1), 'YData', agent_pos(2), ...
%         'ZData', interp2(X, Y, Z, agent_pos(1), agent_pos(2)));
% 
%     % Check if agent reached goal
%     if norm(agent_pos - goal) < 0.5
%         disp('Agent reached the goal!');
%         break;
%     end
% 
%     % Avoid obstacles by applying a repulsion force (optional)
%     % Uncomment the following block for obstacle avoidance:
%     % for i = 1:size(obstacles, 1)
%     %     obs_vec = agent_pos - obstacles(i, :);
%     %     obs_dist = norm(obs_vec);
%     %     if obs_dist < 2
%     %         repulsion = (2 - obs_dist) * obs_vec / obs_dist^3;
%     %         agent_pos = agent_pos + repulsion;
%     %     end
%     % end
% 
%     % Pause to visualize
%     pause(0.05);
% end
% 
% disp('Simulation complete.');
% hold off;
