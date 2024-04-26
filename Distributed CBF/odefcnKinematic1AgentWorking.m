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

    % Nominal control
    % u0 = [-11*cos(10*t), 10*cos(10*t);
    %       11*sin(10*t), 10*sin(10*t)];

    % u0 = [-50*cos(10*t), 50*cos(10*t);
    %       50*sin(10*t), 50*sin(10*t)];
    u0 = [-0.5*cos(10*t), -0.5*cos(10*t);
          0.5*sin(10*t), -0.5*sin(10*t)];
    % u0 = reshape(u0, dimensions*N_a, 1);

    %% Dynamic model
    u = zeros(dimensions, N_a);
    % for i = 1:1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%1:N_a
    %     a_ij = [];
    %     b_ij = [];
    %     for j = 1:N_a
    %         if i ~= j
    %             eta_ij = p(1 : dimensions, i) - p(1 : dimensions, j);
    %             v_ij = p(dimensions+1 : 2*dimensions, i) - p(dimensions+1 : 2*dimensions, j);
    %             a_ij = [a_ij; 2*v_ij.'*v_ij + 2*l1*eta_ij.'*v_ij + l0*(eta_ij.'*eta_ij - 4*agent_radius^2)];
    %             b_ij = [b_ij; 2*eta_ij.'];
    %         end
    %     end
    %     A = -b_ij;
    %     b = mu*a_ij;
    %     H = eye(dimensions);
    %     f = -2*u0(:,i);
    % 
    %     u(:,i) = quadprog(H, f, A, b); 
    % end

    for i = 1:N_a
        h = p(1,i)^2+p(2,i)^2 - (2*agent_radius)^2;
        L_fh = 0;
        L_gh = [2/m*p(1,i), 2/m*p(2,i)];
    
        A = -L_gh;
        b = L_fh + gamma_cbf*h;
    
        H = 2*eye(dimensions);
        f = -2*u0(:,i);
        u(:,i) = quadprog(H, f, A, b);
    end
    u_nom=[u_nom, reshape(u0, dimensions*N_a, 1)];
    u_save=[u_save, reshape(u, dimensions*N_a, 1)];

    %% Dynamics of the system
    dpdt(1,:) = 1/m*u(1,:);                           
    dpdt(2,:) = 1/m*u(2,:);
    dpdt = reshape(dpdt, [], 1);
end

% function dpdt = odefcn(t,p)           DYNAMIC
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
%     mu = 0.5;
% 
%     % Nominal control
%     % u0 = [-11*cos(10*t), 10*cos(10*t);
%     %       11*sin(10*t), 10*sin(10*t)];
% 
%     % u0 = [-50*cos(10*t), 50*cos(10*t);
%     %       50*sin(10*t), 50*sin(10*t)];
%     u0 = [10*sin(10*t), 0;
%           10*sin(10*t), 0];
%     % u0 = reshape(u0, dimensions*N_a, 1);
% 
%     %% Dynamic model
%     u = zeros(dimensions, N_a);
%     % for i = 1:1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%1:N_a
%     %     a_ij = [];
%     %     b_ij = [];
%     %     for j = 1:N_a
%     %         if i ~= j
%     %             eta_ij = p(1 : dimensions, i) - p(1 : dimensions, j);
%     %             v_ij = p(dimensions+1 : 2*dimensions, i) - p(dimensions+1 : 2*dimensions, j);
%     %             a_ij = [a_ij; 2*v_ij.'*v_ij + 2*l1*eta_ij.'*v_ij + l0*(eta_ij.'*eta_ij - 4*agent_radius^2)];
%     %             b_ij = [b_ij; 2*eta_ij.'];
%     %         end
%     %     end
%     %     A = -b_ij;
%     %     b = mu*a_ij;
%     %     H = eye(dimensions);
%     %     f = -2*u0(:,i);
%     % 
%     %     u(:,i) = quadprog(H, f, A, b); 
%     % end
% 
%     L_f2h = 2*p(3,1) + 2*p(4,1) - 2*d/m*(p(1,1)*p(3,1)+p(2,1)*p(4,1));
%     L_gL_fh = [2/m*p(1,1), 2/m*p(2,1)];
%     L_fh = 2*p(1,1)*p(3,1) + 2*p(2,1)*p(4,1);
%     h = p(1,1)^2+p(2,1)^2 - (2*agent_radius)^2;
% 
%     A = -L_gL_fh;
%     b = L_f2h + l1*L_fh + l0*h;
% 
%     H = eye(dimensions);
%     f = -2*u0(:,1);
%     u(:,1) = quadprog(H, f, A, b);
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
%     u0 = [-11*cos(10*t), 10*cos(10*t);
%           11*sin(10*t), 10*sin(10*t)];
% 
%     % u0 = [-50*cos(10*t), 50*cos(10*t);
%     %       50*sin(10*t), 50*sin(10*t)];
%     % u0 = [-11*cos(10*t), 0;
%     %       11*sin(10*t), 0];
%     % u0 = reshape(u0, dimensions*N_a, 1);
% 
%     %% Dynamic model
%     u = zeros(dimensions, N_a);
%     for i = 1:N_a
%         a_ij = [];
%         b_ij = [];
%         for j = 1:N_a
%             if i ~= j
%                 eta_ij = p(1 : dimensions, i) - p(1 : dimensions, j);
%                 v_ij = p(dimensions+1 : 2*dimensions, i) - p(dimensions+1 : 2*dimensions, j);
%                 a_ij = [a_ij; 2*v_ij.'*v_ij + 2*l1*eta_ij.'*v_ij + l0*(eta_ij.'*eta_ij - 4*agent_radius^2)];
%                 b_ij = [b_ij; 2*eta_ij.'];
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