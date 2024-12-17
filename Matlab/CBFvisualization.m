%% Used to visualize barrier function for presentation

close all
clear all

% Define the grid for the 2D plane
x = linspace(-3, 3, 2000);
y = linspace(-3, 3, 2000);
[X, Y] = meshgrid(x, y);

% Generate an organic cloud-like shape using a Gaussian mixture
rng(44); % For reproducibility
num_gaussians = 16; % Number of Gaussian blobs
amplitudes = rand(1, num_gaussians) * 1.5 + 1; % Random amplitudes
centers_x = (rand(1, num_gaussians) -0.5)* 3+0.3; % Random x centers (focused near 0)
centers_y = (rand(1, num_gaussians) -0.5)* 3-0.5; % Random y centers (focused near 0)
widths = rand(1, num_gaussians) * 2 + 0.3; % Random widths

% Compute the combined Gaussian mixture as the safe set
safe_set = zeros(size(X));
for i = 1:num_gaussians
    safe_set = safe_set + amplitudes(i) * ...
        exp(-((X - centers_x(i)).^2 + (Y - centers_y(i)).^2) / (2 * widths(i)^2));
end

% Threshold to create a single connected safe region
threshold = 0.4 * max(safe_set(:));
safe_set_binary = safe_set > threshold;

% Ensure the safe set is connected using morphological operations
safe_set_binary = imfill(safe_set_binary, 'holes'); % Fill holes
safe_set_binary = bwareafilt(safe_set_binary, 1); % Keep only the largest connected region

% Compute the distance to the boundary
[distance, ~] = bwdist(~safe_set_binary);

% Define the potential function (positive within the safe set, zero on boundary)
potential = max(0, distance - 1); % Subtract 1 to make the boundary zero

reciprocal_potential = 1 ./ potential;  % Compute the reciprocal of the potential

% Normalize the potential field to the range [0, 1]
potential_min = min(potential(:));  % Find the minimum value of the potential
potential_max = max(potential(:));  % Find the maximum value of the potential
potential = (potential - potential_min) / (potential_max - potential_min);  % Remap to [0, 1]

% Clip the reciprocal potential between 0 and 1
reciprocal_potential = min(max(reciprocal_potential, 0), 1);  % Clip values between 0 and 1

potential = reciprocal_potential;

%% Plot results
close all
figure('Position', [100 50 800 700]);  % Left Bottom Width Height
surf(X, Y, potential, 'EdgeColor', 'none', 'HandleVisibility','off');  % Plot surface
colormap(parula); % Use the parula colormap
grid on;

% Override colorbar ticks and labels
clim([0 1]);  % Ensure color scale goes from 0 to 1
% clim([-1 0]);  % Ensure color scale goes from 0 to 1
cb = colorbar;
cb.Ticks = linspace(0, 1, 5); % Set colorbar ticks evenly
% cb.Ticks = linspace(-1, 0, 5); % Set colorbar ticks evenly
cb.TickLabels = linspace(0, 1, 5); % Override labels from 0 to 1

% Adjust z-axis ticks and labels
ax = gca;
ax.ZTick = linspace(0, 1, 5); % Set z-axis ticks evenly
ax.ZTickLabel = linspace(0, 1, 5); % Override labels to [0, 1]

% Set axis limits
xlim([-3 3]); ylim([-3 3]); zlim([0 1]);  % Ensure z-axis is between 0 and 1
% xlim([-3 3]); ylim([-3 3]); zlim([-1 0]);  % Ensure z-axis is between 0 and 1


% Customize axis labels and title
xlabel('$x_1$', 'Interpreter','latex', 'FontSize', 22);
ylabel('$x_2$', 'Interpreter','latex', 'FontSize', 22);
zlabel('$h(\mathbf{x})$', 'Interpreter','latex', 'FontSize', 20);
title('Reciprocal Control Barrier Function $B(\mathbf{x})$', 'Interpreter','latex', 'FontSize', 22);
% title('Control Barrier Function $h(\mathbf{x})$', 'Interpreter','latex', 'FontSize', 22);
view(0, 90); % View from above

% Add a small black line on the boundary
hold on;
contour(X, Y, safe_set_binary, [0.5 0.5], 'LineColor', 'k', 'LineWidth', 1.5);  % Boundary at 0.5 level
hold off;

legend('$h(\mathbf{x})=0$', ...
       'Location', 'northwest', ...
       'Interpreter', 'latex', ...
       'FontSize', 18,...
       'BackgroundAlpha',1);
