function Figure_plot()
global Initial_position Goal_position Obstacle1_center Obstacle2_center Obstacle3_center
global x_APF 

% load('x_CBF.mat')

%% Draw two obstacles
radius =0.5;
% Define the circle equation
theta = linspace(0, 2*pi, 100);
Obs1_x= Obstacle1_center(1) + radius*cos(theta);
Obs1_y= Obstacle1_center(2) + radius*sin(theta);
Obs2_x= Obstacle2_center(1) + radius*cos(theta);
Obs2_y= Obstacle2_center(2) + radius*sin(theta);
Obs3_x= Obstacle3_center(1) + radius*cos(theta);
Obs3_y= Obstacle3_center(2) + radius*sin(theta);

figure(2)
% Initial position and goal position
plot(Goal_position(1), Goal_position(2), 'bp', 'MarkerSize', 15, 'MarkerFaceColor', 'b');
hold on
plot(Initial_position(1), Initial_position(2), 'r.', 'MarkerSize', 35, 'MarkerFaceColor', 'r');
% Plot Trajectory
% plot(x_CBF(:,1), x_CBF(:,2), 'b', 'LineWidth', 2, 'MarkerSize', 15, 'MarkerFaceColor', 'k');
plot(x_APF(:,1), x_APF(:,2), 'r', 'LineWidth', 2, 'MarkerSize', 15, 'MarkerFaceColor', 'k');


% Obstacles
fill(Obs1_x, Obs1_y, 'red', 'FaceAlpha', 0.3, 'EdgeColor', 'red');  % 'red' for fill and boundary color
fill(Obs2_x, Obs2_y, 'red', 'FaceAlpha', 0.3, 'EdgeColor', 'red');  % 'red' for fill and boundary color
fill(Obs3_x, Obs3_y, 'red', 'FaceAlpha', 0.3, 'EdgeColor', 'red');  % 'red' for fill and boundary color
plot(Obstacle1_center(1),Obstacle1_center(2),'k.','MarkerSize',15)
plot(Obstacle2_center(1),Obstacle2_center(2),'k.','MarkerSize',15)
plot(Obstacle3_center(1),Obstacle3_center(2),'k.','MarkerSize',15)

% %Add Arrow
% % Add an arrow using quiver with a larger arrowhead
% arrowLength = 0.55;  % Length of the arrow
% arrowDirection = [1, 0];  % Direction of the arrow
% arrowHeadSize = 0.5;  % Size of the arrowhead
% quiver(Obstacle1_center(1), Obstacle1_center(2), arrowLength * arrowDirection(1), arrowLength * arrowDirection(2), 'k', 'LineWidth', 2, 'MaxHeadSize', arrowHeadSize);
% quiver(Obstacle2_center(1), Obstacle2_center(2), arrowLength * arrowDirection(1), arrowLength * arrowDirection(2), 'k', 'LineWidth', 2, 'MaxHeadSize', arrowHeadSize);
text(Obstacle1_center(1)-0.33, Obstacle1_center(2)-0.25, 'Obs. 1', 'FontSize', 20, 'Color', 'k','interpreter','latex');
% text(Obstacle1_center(1)-0.35, Obstacle1_center(2)+0.25, '$r=0.5$', 'FontSize', 15, 'Color', 'k','interpreter','latex');
text(Obstacle2_center(1)-0.33, Obstacle2_center(2)-0.25, 'Obs. 2', 'FontSize', 20, 'Color', 'k','interpreter','latex');
% text(Obstacle2_center(1)-0.35, Obstacle2_center(2)+0.25, '$r=0.5$', 'FontSize', 15, 'Color', 'k','interpreter','latex');
text(Obstacle3_center(1)-0.33, Obstacle3_center(2)-0.25, 'Obs. 3', 'FontSize', 20, 'Color', 'k','interpreter','latex');
% text(Obstacle3_center(1)-0.35, Obstacle3_center(2)+0.25, '$r=0.5$', 'FontSize', 15, 'Color', 'k','interpreter','latex');
set(gca,'FontSize',20)
set(gcf,'Position',[200,200,1000,725], 'color','w')
axis equal
grid on


end

