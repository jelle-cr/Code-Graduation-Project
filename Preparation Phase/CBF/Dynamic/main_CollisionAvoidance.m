close all
clear all

% Objective: Trajectory tracking and Collision Avoidance with CLF+CBF
global u_save

% Load system parameters
par = Initial_Parameter();
m = par.m;
d = par.d;
p_d = par.p_d;


% Initial conditions
% p0 = [0; 0; 0; 0];  % initial position and velocity
p0 = [0; 0; 0; 0];

% Time vector
t_span = 0:0.01:10;  % simulation time

[p] = ode4(@odefcn,t_span,p0);

u = zeros(height(u_save), length(p));
for i = 1:length(p)
    u(:,i) = u_save(:,1+(i-1)*4);
end

%% Plot results
font = 14;
close all
figure;
plot(p(:, 1), p(:, 2),LineWidth=2); hold on;
scatter(p0(1), p0(2), 60, 'o','MarkerEdgeColor','blue', 'MarkerFaceColor', 'none',LineWidth=2);
scatter(p(end,1), p(end,2), 80, 'x','MarkerEdgeColor','red', 'MarkerFaceColor', 'none',LineWidth=2);
% Plotting of the unsafe regions
theta = 0:0.01:2*pi;
for i = 1:height(par.ob)
    circle_x = par.r * cos(theta) + par.ob(i,1);
    circle_y = par.r * sin(theta) + par.ob(i,2);
    patch(circle_x, circle_y, 'red', 'FaceAlpha', 0.25, 'EdgeColor', 'red'); 
    text(par.ob(i,1), par.ob(i,2), ['$ob_', num2str(i),'$'], 'HorizontalAlignment', 'center',Interpreter='latex',FontSize=font); 
end
xlabel('X Position [m]',Interpreter='latex',FontSize=font);
ylabel('Y Position [m]',Interpreter='latex',FontSize=font);
title('Quadcopter Trajectory',Interpreter='latex',FontSize=font);
grid on;
axis('equal')
set(gcf, 'Position', [200 300 500 400]); % [left bottom width height]

figure;
subplot(3,1,1);
plot(t_span, p,LineWidth=2); hold on;
scatter(zeros(size(par.p_d)),par.p_d, 'x','black')
% plot(t_span, p(:, 3),LineWidth=2);
% plot(t_span, p(:, 4),LineWidth=2);
xlabel('Time [s]',Interpreter='latex',FontSize=font);
ylabel('State',Interpreter='latex',FontSize=font);
title('Quadcopter States',Interpreter='latex',FontSize=font);
legend({'$x$','$y$','$\dot{x}$','$\dot{y}$'},Interpreter="latex",FontSize=font);
% legend({'$x$', '$y$'},Interpreter="latex",FontSize=font);
grid on;

subplot(3,1,2);
plot(t_span, u(1:2,:),LineWidth=2); hold on;
xlabel('Time [s]',Interpreter='latex',FontSize=font);
ylabel('Input',Interpreter='latex',FontSize=font);
title('Quadcopter Inputs',Interpreter='latex',FontSize=font);
legend({'$u_x$','$u_y$'},Interpreter="latex",FontSize=font);
grid on;

% Compute and plot CBF
h = zeros(height(par.ob),length(t_span));
for i = 1:height(par.ob)
    h(i,:) = ((p(:,1)-par.ob(i,1)).^2+(p(:,2)-par.ob(i,2)).^2).^(1/2) - par.r;
end
subplot(3,1,3);
plot(t_span, h,LineWidth=2); hold on;
xlabel('Time [s]',Interpreter='latex',FontSize=font);
ylabel('$h(p)$',Interpreter='latex',FontSize=font);
title('Control Barrier Functions',Interpreter='latex',FontSize=font);
leg = {};
for i = 1:height(par.ob)
    leg{end + 1} = ['$ob_', num2str(i), '$'];
end
legend(leg,Interpreter='latex',FontSize=font);
grid on;
set(gcf, 'Position', [800 0 600 800]); % [left bottom width height]

figure;
plot(t_span, u(3, :),LineWidth=2);
ylabel('$\delta$',Interpreter='latex',FontSize=font);
xlabel('Time [s]',Interpreter='latex',FontSize=font);
title('Relaxation Parameter',Interpreter='latex',FontSize=font);
grid on;
set(gcf, 'Position', [1400 300 500 300]); % [left bottom width height]