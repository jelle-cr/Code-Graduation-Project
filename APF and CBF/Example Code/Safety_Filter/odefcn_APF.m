function dxdt = odefcn_APF(t,x)
global Goal_position u_APF_save
global Obstacle1_center Obstacle2_center Obstacle3_center
% Parameters
K_att = 1;
K_rep = 0.001;
D_obs = 0.5;
rho_0 =0.1;

t

%% Dynamics -first case
dxdt = zeros(2,1);
% Attractive Potential Field and Force
U_att=1/2*K_att*(x-Goal_position).'*(x-Goal_position);
%Define nominal control input
f_x=[x(2),x(1)].';
g_x=[1,0;0,1];
a_x=(x-Goal_position).'*f_x;
b_x=(x-Goal_position).'*g_x;
sigma_x=norm(b_x)^2;
ax_tilde=a_x+sigma_x;
if ax_tilde<0
    u_nom=[0,0].';
end
if ax_tilde==0 && norm(b_x)==0
    u_nom=[0,0].';
end
if ax_tilde>=0 && norm(b_x)~=0
    u_nom=-ax_tilde/(norm(b_x)^2)*b_x.';
end

% u=u_nom;
%We need to identify the obstacle that is currently causing us to avoid collisions
Dist_1=norm(x-Obstacle1_center);
Dist_2=norm(x-Obstacle2_center);
Dist_3=norm(x-Obstacle3_center);
Min_Dist=min([Dist_1,Dist_2,Dist_3]);
if Min_Dist==Dist_1
    Obstacle_center=Obstacle1_center;
end
if Min_Dist==Dist_2
    Obstacle_center=Obstacle2_center;
end
if Min_Dist==Dist_3
    Obstacle_center=Obstacle3_center;
end
rho_x = norm(x-Obstacle_center) -D_obs;
if rho_x>rho_0
    F_rep=[0,0].';
else
    F_rep=-K_rep/(rho_x^2)*(1/rho_x-1/rho_0)*(x-Obstacle_center)/norm(x-Obstacle_center);
end
c_x=F_rep.'*f_x;
d_x=F_rep.'*g_x;
cx_tilde=c_x+norm(F_rep)^2;%   -d_x*u_nom;%%%%%%%%%%%%%%%%%%%%%%%%%%%
phi_var=cx_tilde+d_x*u_nom;
% if phi_var<0
%     u=u_nom;
% end
% if phi_var==0 && norm(d_x)==0
%    u=u_nom;  
% end
% if phi_var>=0 && norm(d_x)~=0
%  u=-phi_var/(norm(d_x)^2)*d_x.';%       +u_nom;%%%%%%%%%%%%%%  
% end

if rho_x>rho_0
    u=u_nom;
else
    u=-phi_var/(norm(d_x)^2)*d_x.';%       +u_nom;%%%%%%%%%%%%%%  
end

%%%%%%%%%%%%%%%%%%
% u = u_nom;

dxdt(1) =x(2)+u(1);
dxdt(2) =x(1)+u(2);
u_APF_save=[u_APF_save,u];
end