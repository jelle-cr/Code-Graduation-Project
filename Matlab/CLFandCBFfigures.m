clear all
close all

% Create a mesh grid for 3D surface
[x, y] = meshgrid(linspace(-2, 2, 100));
z = x.^2 + y.^2;  % Convex bowl function (Lyapunov function)

% Create the figure
figure;
surf(x, y, z, 'FaceAlpha', 0.4, 'EdgeColor', 'none');
colormap('parula');
hold on;
grid on;

% Add contours on the XY plane
% contour3(x, y, z, 20, 'k');

% Function for a spiraling trajectory
theta = linspace(0, 10*pi, 500);   % Angular range (number of turns)
spiral_rate = 0.1;                % Adjustable spiral rate (controls how quickly it converges)
r = 2*exp(-spiral_rate * theta);    % Exponential decay for the radius
x_spiral = r .* cos(theta);       % X-coordinates of the spiral
y_spiral = r .* sin(theta);       % Y-coordinates of the spiral
z_spiral = x_spiral.^2 + y_spiral.^2;  % Corresponding Z-coordinates (on the Lyapunov surface)

% Plot the spiraling trajectory
plot3(x_spiral, y_spiral, z_spiral, 'r-', 'LineWidth', 3);

% Mark the starting point of the spiral
plot3(x_spiral(1), y_spiral(1), z_spiral(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');


% Adjust the view angle
view(45, 30);

% Set axis limits
xlim([-2 2]);
ylim([-2 2]);
zlim([0 4]);

% Add color bar
colorbar;

% set(gcf, 'Renderer', 'painters')
% exportgraphics(gcf, 'my_3d_plot.pdf', 'ContentType', 'vector');

