close all

% Define the grid and the potential function U
[x, y] = meshgrid(-5:0.1:5, -5:0.1:5);
U_base = x.^2 + y.^2; % Base potential: Parabolic bowl

% Define obstacle with a steep repulsive potential
obs_center = [1, 1]; % Center of the obstacle
obs_radius = 0.5; % Radius of the repulsive obstacle
U_rep = 10 ./ sqrt((x - obs_center(1)).^2 + (y - obs_center(2)).^2 + 0.1); % Repulsive potential
U_rep(U_rep > 500) = 500; % Cap the maximum value of the repulsive potential

% Combine potentials
U = U_base + U_rep;

% Compute the gradient (negative for force)
[Ux, Uy] = gradient(U, 0.1, 0.1); % Gradients of U w.r.t. x and y
Fx = -Ux; % Force in x-direction
Fy = -Uy; % Force in y-direction

% Initial position of the ball
pos = [2, 3]; % Initial (x, y) position of the ball
z = interp2(x, y, U, pos(1), pos(2)); % Get initial z-coordinate from U

% Ball radius
r_a = 0.3;

% Set up the figure
figure;
surf(x, y, U, 'EdgeColor', 'none'); % Surface plot
hold on;
colormap jet;
shading interp;

% Create the ball as a sphere
[bx, by, bz] = sphere(20); % Sphere coordinates
ballHandle = surf(r_a * bx + pos(1), r_a * by + pos(2), r_a * bz + z, ...
    'FaceColor', 'r', 'EdgeColor', 'none'); % Ball as a surface

xlabel('X');
ylabel('Y');
zlabel('Potential U');
title('Ball Rolling Down the Potential Surface with Obstacle');
trajectory = animatedline('Color', 'r', 'LineWidth', 2); % Trajectory of the ball

% Simulation parameters
dt = 0.05; % Time step
angle = 30; % Initial azimuthal angle for dynamic view
view(angle, 60); % Initial view angle

% Simulation loop
for t = 0:dt:10
    % Interpolate force at the current position
    fx = interp2(x, y, Fx, pos(1), pos(2));
    fy = interp2(x, y, Fy, pos(1), pos(2));
    
    % Update position based on the force
    pos(1) = pos(1) + fx * dt;
    pos(2) = pos(2) + fy * dt;
    
    % Get new height (z-coordinate) from the potential
    z = interp2(x, y, U, pos(1), pos(2));
    
    % Update ball position (move the sphere)
    set(ballHandle, ...
        'XData', r_a * bx + pos(1), ...
        'YData', r_a * by + pos(2), ...
        'ZData', r_a * bz + z);
    addpoints(trajectory, pos(1), pos(2), z); % Add point to trajectory
    
    % Update the view angle for dynamic effect
    % angle = angle + 0.5; % Increment the azimuthal angle
    view(angle, 60); % Update the view
    
    % Pause for animation effect
    pause(0.05);
    
    % Break the loop if the ball reaches a low-gradient region
    if norm([fx, fy]) < 1e-3
        break;
    end
end

hold off;
