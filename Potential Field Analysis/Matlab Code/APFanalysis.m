close all
clear all

n = 500;
x = linspace(-3,3,n);
y = linspace(-3,3,n);

r_a = 0.5;
r_o = 0.4;

K_att = 1;
N_a = 1;
p_d = rand(2,N_a)*(3+3)-3;
U_att = zeros(length(x), length(y));
F_att = zeros(length(x), length(y), height(p_d));

K_rep = 0.001;
rho_0 = 0.5;
N_o = 15;
p_o = rand(2,N_o)*(3+3)-3;
U_rep = zeros(length(x), length(y));
F_rep = zeros(length(x), length(y), height(p_d));

for i = 1:length(x)
    for j = 1:length(y)
        p = [x(i); y(j)];
        U_att(i,j) = 1/2*K_att*norm(p - p_d);
        F_att(i,j,:) = K_att*(p - p_d);

        for o = 1:N_o
            rho = norm(p - p_o(:,o)) - r_a - r_o;   
            if rho < rho_0
                rho = max(rho, 1e-6);           % In order to fill in the cylinder
                U_rep(i,j) = U_rep(i,j) + 1/2*K_rep*(1/rho - 1/rho_0)^2;
                if rho > 1e-1                   % Only calculate this outside of the obstacles, otherwise we get scaling issues
                    F_rep(i,j,:) = squeeze(F_rep(i,j,:)) -K_rep/rho^2*(1/rho - 1/rho_0)*(p - p_o(:,o))/norm(p - p_o(:,o));
                else
                    F_att(i,j,:) = [0; 0];      % We don't want any quivers to be drawn near the obstacle
                end
            end
        end
    end
end

Potential = U_att + U_rep;
Potential = min(Potential, 1*max(max(U_att)));
Force = -F_att - F_rep;

minPotential = min(min(Potential));
[globalMinX_idx, globalMinY_idx] = find(Potential == minPotential);
globalMinX = x(globalMinX_idx);
globalMinY = y(globalMinY_idx);

% Find local minima using imregionalmin
localMinima = imregionalmin(Potential);     % Way faster than looping through matrix manually

% Get the indices of the local minima
[localMinX_idx, localMinY_idx] = find(localMinima);

% Convert indices to coordinates in the x and y arrays
localMinX = x(localMinX_idx);
localMinY = y(localMinY_idx);

%% Plotting
% Surface plot
figure('Position', [100 50 800 700]);  %Left Bottom Width Height
surf(x,y,Potential','FaceAlpha',1, 'EdgeColor','none')
ax = gca;
set(ax, 'FontSize', 12);
xlabel('$x$ [m]', 'Interpreter','latex', 'FontSize', 16);
ylabel('$y$ [m]', 'Interpreter','latex', 'FontSize', 16);
zlabel('Potential', 'Interpreter','latex', 'FontSize', 16);


% Contour/quiver plot
figure('Position', [1000 50 800 700]);  %Left Bottom Width Height
hold on
% step = floor(n/30);                % Subsample the amount of quiver vectors
% quiver(x(1:step:end, 1:step:end), y(1:step:end, 1:step:end), Force(1:step:end, 1:step:end, 1)', Force(1:step:end, 1:step:end, 2)', 0.8, 'HandleVisibility', 'off');

% Plot contour lines
contour(x,y,Potential', 20, 'HandleVisibility', 'off');
colorbar

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

% Plot Local Minima
for i = 1:length(localMinX)
    if abs(localMinX(i) - globalMinX) > 1e-6 || abs(localMinY(i) - globalMinY) > 1e-6      % Exclude the Global Minimum
        plot(localMinX(i), localMinY(i), 'o', 'MarkerSize', 8, ...
                                 'MarkerEdgeColor', 'green', ...
                                 'MarkerFaceColor', 'green', ...
                                 'HandleVisibility', 'off');
    else
        plot(NaN, NaN, 'o', 'MarkerEdgeColor', 'green', ...
                            'MarkerFaceColor', 'green', ...
                        	'MarkerSize', 8, ...
                            'DisplayName', 'Local Minimum');
    end
end

% Plot Global minimum
plot(globalMinX, globalMinY, 'o', 'MarkerSize', 8, ...
                                        'MarkerEdgeColor', 'blue', ...
                                        'MarkerFaceColor', 'blue', ...
                                        'DisplayName', 'Global Minimum');

% Plot Goal
plot(p_d(1), p_d(2), 'x','MarkerSize', 12, ...
                         'MarkerEdgeColor', 'red', ...
                         'LineWidth', 2,...
                         'DisplayName', 'Desired Position');

ax = gca;
set(ax, 'FontSize', 12);
xlabel('$x$ [m]', 'Interpreter','latex', 'FontSize', 16);
ylabel('$y$ [m]', 'Interpreter','latex', 'FontSize', 16);
legend('Location', 'northwest', 'BackgroundAlpha', 0.3, 'Interpreter', 'latex', 'FontSize', 14);
axis equal
xlim([-3, 3]);
ylim([-3, 3]);