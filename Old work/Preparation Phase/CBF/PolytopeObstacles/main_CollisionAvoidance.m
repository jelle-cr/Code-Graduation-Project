close all
clear all

% Objective: Trajectory tracking and Collision Avoidance with CLF+CBF
global u_save

% Load system parameters
t=0;
par = Initial_Parameter(t);
m = par.m;
d = par.d;
p_d = par.p_d;
ob = par.ob;

% Determine obstacle vertices
polyPoints = ob;
polyLines = cell(1,width(polyPoints));
for i = 1:width(polyPoints)
    polyLines{i} = zeros(height(polyPoints{i}),3);
    polyPoints{i} = [polyPoints{i}; polyPoints{i}(1:2,:)];
    
    for j = 1:height(polyLines{i})
        m = (polyPoints{i}(j+1,2)-polyPoints{i}(j,2)) / (polyPoints{i}(j+1,1)-polyPoints{i}(j,1));  % Line direction
        if m > 10000
            m = 1;
        elseif m < -10000
            m = -1;
        end
        b = -1;
        c = polyPoints{i}(j,2)-m*polyPoints{i}(j,1);
        polyLines{i}(j,1) = m;
        polyLines{i}(j,2) = b;
        polyLines{i}(j,3) = c;
        
        nextPoint = m*polyPoints{i}(j+2,1) + b*polyPoints{i}(j+2,2) + c;
        if nextPoint >= 0
            polyLines{i}(j,:) = -polyLines{i}(j,:);
        end
    end
end
% polyLines = polyEdges
save('polytopeLines.mat','polyLines');

%%
% Initial conditions
% p0 = [0; 0; 0; 0];  % initial position and velocity
p0 = [0; 0];

% Time vector
t_span = 0:0.01:6;  % simulation time

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
for i = 1:width(ob)
    patch([ob{i}(:,1); ob{i}(1,1)], [ob{i}(:,2); ob{i}(1,2)], 'red', 'FaceAlpha', 0.25, 'EdgeColor', 'red'); 
    text((ob{i}(1,1)+ob{i}(2,1))/2, (ob{i}(1,2)+ob{i}(3,2))/2, ['$ob_', num2str(i),'$'], 'HorizontalAlignment', 'center',Interpreter='latex',FontSize=font); 
end
xlabel('X Position [m]',Interpreter='latex',FontSize=font);
ylabel('Y Position [m]',Interpreter='latex',FontSize=font);
title('Quadcopter Trajectory',Interpreter='latex',FontSize=font);
grid on;
axis('equal')
set(gcf, 'Position', [200 300 500 400]); % [left bottom width height]

figure;
subplot(3,1,1);
plot(t_span, p(:, 1),LineWidth=2); hold on;
plot(t_span, p(:, 2),LineWidth=2);
scatter(zeros(size(par.p_d)),par.p_d, 'x','black')
% plot(t_span, p(:, 3),LineWidth=2);
% plot(t_span, p(:, 4),LineWidth=2);
xlabel('Time [s]',Interpreter='latex',FontSize=font);
ylabel('State [m]',Interpreter='latex',FontSize=font);
title('Quadcopter States',Interpreter='latex',FontSize=font);
% legend({'$x$','$y$','$\dot{x}$','$\dot{y}$'},Interpreter="latex",FontSize=12);
legend({'$x$', '$y$'},Interpreter="latex",FontSize=font);
grid on;

subplot(3,1,2);
plot(t_span, u(1:2,:),LineWidth=2); hold on;
xlabel('Time [s]',Interpreter='latex',FontSize=font);
ylabel('Input',Interpreter='latex',FontSize=font);
title('Quadcopter Inputs',Interpreter='latex',FontSize=font);
legend({'$u_x$','$u_y$'},Interpreter="latex",FontSize=font);
grid on;

% Compute and plot CBF
h = zeros(width(polyLines),length(t_span));
for i = 1:width(polyLines)
    lineBarriers = zeros(height(polyLines{i}),length(t_span));
    for j = 1:height(polyLines{i})
        lineBarriers(j,:) = polyLines{i}(j,1)*p(:,1) + polyLines{i}(j,2)*p(:,2) + polyLines{i}(j,3);
    end
    [max_value, max_index] = max(lineBarriers);
    h(i,:) = max_value;
end

subplot(3,1,3);
plot(t_span, h,LineWidth=2); hold on;
xlabel('Time [s]',Interpreter='latex',FontSize=font);
ylabel('$h(p)$',Interpreter='latex',FontSize=font);
title('Control Barrier Functions',Interpreter='latex',FontSize=font);
leg = {};
for i = 1:height(h)
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
% legend({'$\lambda$'},Interpreter="latex");
grid on;
set(gcf, 'Position', [1400 300 500 300]); % [left bottom width height]