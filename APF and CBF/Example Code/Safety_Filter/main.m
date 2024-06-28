clear all
close all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Research Problem: Artificial Potential Field versus CLF-CBF-QP
%Author: Ming Li
%Date: March 14 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Initial_position Goal_position Obstacle1_center Obstacle2_center Obstacle3_center
global x_APF u_APF_save 
Initial_position(:,1)=[0,0].';
Goal_position(:,1)=[3,5].';
Obstacle1_center=[1,1.5].';
Obstacle2_center=[2.5,3].';
Obstacle3_center=[4,4.2].';
t_end =10;
t_span=[0:0.01:t_end];
%% Artificial Potential Field
for i=1:size(Initial_position,2)
    % [T,x_APF(:,:,i)] = ode45(@odefcn_APF,t_span,Initial_position(:,i));
    [x_APF(:,:,i)] = ode4(@odefcn_APF,t_span,Initial_position(:,i));
end

save x_SF_01 x_APF
% % Parameters
% K_att = 1;
% K_rep = 1;
% D_obs = 0.5;
% rho_0 =0.2;
% for i=1:size(x_APF,1)
%     x=x_APF(i,:).';
%     F_att=K_att*(x-Goal_position);
%     if norm(x-Obstacle1_center)>norm(x-Obstacle2_center)
%         Obstacle_center=Obstacle2_center;
%     else
%         Obstacle_center=Obstacle1_center;
%     end
%     rho_x = norm(x-Obstacle_center) -D_obs;
%     % Repulsive Potential Field and Force
%     if rho_x>rho_0
%         U_rep=0;
%     else
%         U_rep=1/2*K_rep*(1/rho_x-1/rho_0)^2;
%     end
%     
%     if rho_x>rho_0
%         F_rep=[0,0].';
%     else
%         F_rep=-K_rep/(rho_x^2)*(1/rho_x-1/rho_0)*(x-Obstacle_center)/norm(x-Obstacle_center);
%         %     F_rep=K_rep/(rho_x^2)*(1/rho_x-1/rho_0)*(x-Obstacle_center)/rho_x;
%     end
%     u(i,:)=-F_att-F_rep;
% end
% figure(1)
% plot(vecnorm(u.'),'k--')
%% Plot Figures
Figure_plot()





