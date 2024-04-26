%% Second order (added nominal trajectory)
function dpdt = odefcn(t,p)      
    global u_save u_nom

    load('parameters.mat');

    p = reshape(p, states, N_a);
    dpdt = zeros(states,N_a);
    % Note that [p(1) p(2) p(3) p(4)]==[x y x' y']
    % time
    t

    % Full responsibility mu = 1, or half responsibility mu = 1/2
    mu = 1;

    % Nominal controls
    u0 = sign_rand.*[A_rand; A_rand].*[cos(f_rand*t); sin(f_rand*t)];

    %% Dynamic model
    u = zeros(dimensions, N_a);
    for i = 1:N_a%1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%1:N_a
        A = [];
        b = [];
        for j = 1:N_a
            if i ~= j
                xi_ij = p(1 : dimensions, i) - p(1 : dimensions, j);
                % v_ij = p(dimensions+1 : 2*dimensions, i) - p(dimensions+1 : 2*dimensions, j);
                h = xi_ij.'*xi_ij - (2*agent_radius)^2;
                grad_h = [2*xi_ij; 0; 0];
                L_fh = 2*[p(3,i), p(4,i)]*xi_ij;
                L_f2h = 2*p(3,i)^2 + 2*p(4,i)^2 - 2*d/m*xi_ij.'*[p(3,i); p(4,i)];
                L_gL_fh = 2/m*xi_ij.';

                A = [A; -L_gL_fh];
                b = [b; L_f2h + l1*L_fh + l0*h];
            end
        end
        H = 2*eye(dimensions);
        f = -2*u0(:,i);
        u(:,i) = quadprog(H, f, A, b);
    end

    u_nom=[u_nom, reshape(u0, dimensions*N_a, 1)];
    u_save=[u_save, reshape(u, dimensions*N_a, 1)];

    %% Dynamics of the system
    dpdt(1,:) = p(3,:);                           
    dpdt(2,:) = p(4,:);
    dpdt(3,:) = -d/m*p(3,:) + 1/m * u(1,:);
    dpdt(4,:) = -d/m*p(4,:) + 1/m * u(2,:);
    dpdt = reshape(dpdt, [], 1);
end

% %% Second order
% function dpdt = odefcn(t,p)      
%     global u_save u_nom
% 
%     load('parameters.mat');
% 
%     p = reshape(p, states, N_a);
%     dpdt = zeros(states,N_a);
%     % Note that [p(1) p(2) p(3) p(4)]==[x y x' y']
%     % time
%     t
% 
%     % Full responsibility mu = 1, or half responsibility mu = 1/2
%     mu = 1;
% 
%     % Nominal control
%     u0 = sign_rand.*[A_rand; A_rand].*[cos(f_rand*t); sin(f_rand*t)];
% 
%     %% Dynamic model
%     u = zeros(dimensions, N_a);
%     for i = 1:N_a%1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%1:N_a
%         A = [];
%         b = [];
%         for j = 1:N_a
%             if i ~= j
%                 xi_ij = p(1 : dimensions, i) - p(1 : dimensions, j);
%                 % v_ij = p(dimensions+1 : 2*dimensions, i) - p(dimensions+1 : 2*dimensions, j);
%                 h = xi_ij.'*xi_ij - (2*agent_radius)^2;
%                 grad_h = [2*xi_ij; 0; 0];
%                 L_fh = 2*[p(3,i), p(4,i)]*xi_ij;
%                 L_f2h = 2*p(3,i)^2 + 2*p(4,i)^2 - 2*d/m*xi_ij.'*[p(3,i); p(4,i)];
%                 L_gL_fh = 2/m*xi_ij.';
% 
%                 A = [A; -L_gL_fh];
%                 b = [b; L_f2h + l1*L_fh + l0*h];
%             end
%         end
%         H = 2*eye(dimensions);
%         f = -2*u0(:,i);
%         u(:,i) = quadprog(H, f, A, b);
%     end
% 
%     u_nom=[u_nom, reshape(u0, dimensions*N_a, 1)];
%     u_save=[u_save, reshape(u, dimensions*N_a, 1)];
% 
%     %% Dynamics of the system
%     dpdt(1,:) = p(3,:);                           
%     dpdt(2,:) = p(4,:);
%     dpdt(3,:) = -d/m*p(3,:) + 1/m * u(1,:);
%     dpdt(4,:) = -d/m*p(4,:) + 1/m * u(2,:);
%     dpdt = reshape(dpdt, [], 1);
% end

%% First order
% function dpdt = odefcn(t,p)
%     global u_save u_nom
%     hi = 1
%     load('parameters.mat');
% 
%     p = reshape(p, states, N_a);
%     dpdt = zeros(states,N_a);
%     % Note that [p(1) p(2) p(3) p(4)]==[x y x' y']
%     % time
%     t
% 
%     % Full responsibility mu = 1, or half responsibility mu = 1/2
%     mu = 1;
% 
%     % Nominal control
%     % u0 = [-11*cos(10*t), 10*cos(10*t);
%     %       11*sin(10*t), 10*sin(10*t)];
% 
%     % u0 = [-50*cos(10*t), 50*cos(10*t);
%     %       50*sin(10*t), 50*sin(10*t)];
%     u0 = [-0.5*cos(10*t), 0.5*cos(8*t), -0.5*cos(11*t), 0.5*cos(10*t);
%           0.5*sin(10*t), -0.5*sin(8*t), -0.5*sin(11*t), 0.5*sin(10*t)];
%     % u0 = reshape(u0, dimensions*N_a, 1);
% 
%     %% Kinematic model
%     u = zeros(dimensions, N_a);
%     for i = 1:N_a
%         A = [];
%         b = [];
%         for j = 1:N_a
%             if i ~= j
%                 xi_ij = p(1 : dimensions, i) - p(1 : dimensions, j);
%                 h = xi_ij.'*xi_ij - (2*agent_radius)^2;
%                 grad_h = 2*xi_ij;
%                 L_fh = 0;
%                 L_gh = [dot(grad_h,[1/m; 0]), dot(grad_h,[0; 1/m])];
% 
%                 A = [A; -L_gh];
%                 b = [b; L_fh + gamma_cbf*h];
%             end
%         end
%         H = 2*eye(dimensions);
%         f = -2*u0(:,i);
%         u(:,i) = quadprog(H, f, A, b);
%     end
%     u_nom=[u_nom, reshape(u0, dimensions*N_a, 1)];
%     u_save=[u_save, reshape(u, dimensions*N_a, 1)];
% 
%     %% Dynamics of the system
%     dpdt(1,:) = 1/m*u(1,:);                           
%     dpdt(2,:) = 1/m*u(2,:);
%     dpdt = reshape(dpdt, [], 1);
% end

%% Second order paper
% function dpdt = odefcn(t,p)
%     global u_save u_nom
% 
%     load('parameters.mat');
% 
%     p = reshape(p, states, N_a);
%     dpdt = zeros(states,N_a);
%     % Note that [p(1) p(2) p(3) p(4)]==[x y x' y']
%     % time
%     t
% 
%     % Full responsibility mu = 1, or half responsibility mu = 1/2
%     mu = 1;
% 
%     % Nominal control
%     u0 = [-11*cos(10*t), 0;
%           11*sin(10*t), 0];
% 
%     % u0 = [-50*cos(10*t), 50*cos(10*t);
%     %       50*sin(10*t), 50*sin(10*t)];
%     % u0 = [-11*cos(10*t), 0;
%     %       11*sin(10*t), 0];
%     % u0 = reshape(u0, dimensions*N_a, 1);
% 
%     %% Dynamic model
%     u = zeros(dimensions, N_a);
%     for i = 1:1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%1:N_a
%         a_ij = [];
%         b_ij = [];
%         for j = 1:N_a
%             if i ~= j
%                 xi_ij = p(1 : dimensions, i) - p(1 : dimensions, j);
%                 v_ij = p(dimensions+1 : 2*dimensions, i) - p(dimensions+1 : 2*dimensions, j);
%                 a_ij = [a_ij; 2*v_ij.'*v_ij + 2*l1*xi_ij.'*v_ij + l0*(xi_ij.'*xi_ij - 4*agent_radius^2)];
%                 b_ij = [b_ij; 2*xi_ij.'];
%             end
%         end
%         A = -b_ij;
%         b = mu*a_ij;
%         H = 2*eye(dimensions);
%         f = -2*u0(:,i);
% 
%         u(:,i) = quadprog(H, f, A, b); 
%     end
% 
%     u_nom=[u_nom, reshape(u0, dimensions*N_a, 1)];
%     u_save=[u_save, reshape(u, dimensions*N_a, 1)];
% 
%     %% Dynamics of the system
%     dpdt(1,:) = p(3,:);                           
%     dpdt(2,:) = p(4,:);
%     dpdt(3,:) = -d/m*p(3,:) + 1/m * u(1,:);
%     dpdt(4,:) = -d/m*p(4,:) + 1/m * u(2,:);
%     % dpdt(1,:) = u(1,:);
%     % dpdt(2,:) = u(2,:);
%     dpdt = reshape(dpdt, [], 1);
% end