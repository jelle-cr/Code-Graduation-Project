function dxdt = odefcn(t, x)      
    load('./Data/parameters.mat');

    dxdt = zeros(n, 1);

    %% Dynamics
    f = A*x(:);
    g = B;

    %% Potential fields
    p = x(1:2, :);              % Current position
    p_d = x_d(1:2);             % Desired position
    p_o = x_o(1:2, :);          % Obstacle positions

    [F_att, F_rep, h] = Functions.potential_gradients(m, p, p_d, p_o, k_att, k_rep, r_a, r_o, h_0);

    %% Controller               % Either uses CBF or APF
    u = zeros(m, 1);            % Initialize controller
    u_CLF = zeros(m, 1);          
    u_CBF = zeros(m, 1);          
    u_APF = zeros(m, 1);     

    a = F_att.'*f;
    b = F_att.'*g;
    sigma = norm(b)^2;
    a_tilde = a + sigma;

    if a_tilde > 0
        u_CLF = -a_tilde/norm(b)^2 * b.';
    end

    alpha = 1;
    c = F_rep.'*f - alpha;
    d = F_rep.'*g;
    gamma = norm(d)^2 + alpha - d*u_CLF;
    c_tilde = c + gamma;
    phi = c_tilde + d*u_CLF;

    %% Standard CBF 
    if phi <= 0
        u_CBF = u_CLF;
    else
        u_CBF = u_CLF - phi/norm(d)^2 * d.';
    end

    %% APF variant
    if h >= h_0
        u_APF = u_CLF;
    else
        u_APF = u_CLF - phi/norm(d)^2 * d.';
    end

    if u_CBF ~= u_APF
        % if u_CBF ~= u_CLF
        phi
            t
            u_CLF
            u_CBF
            u_APF
        % end
    end

    %% ODE
    dxdt = A*x + B*u_CBF;
    dxdt = A*x + B*u_APF;
    % dxdt = reshape(dxdt, [], 1);

end



% function dxdt = odefcn(t, x)      
%     load('./Data/parameters.mat');
% 
%     x = reshape(x, n, N_a);
%     dxdt = zeros(n, 1);
% 
%     for i = 1:N_a
%         x = squeeze(x(:,i))
% 
%         %% Dynamics
%         f = A*x(:,1);
%         g = B;
% 
%         %% Potential fields
%         p = x(1:2, :);              % Current position
%         p_d = x_d(1:2);             % Desired position
%         p_o = x_o(1:2, :);          % Obstacle positions
% 
%         [F_att, F_rep, h] = Functions.potential_gradients(m, p, p_d, p_o, k_att, k_rep, r_a, r_o, h_0);
% 
%         %% Controller               % Either uses CBF or APF
%         u = zeros(m, 1);            % Initialize controller
%         u_CLF = zeros(m, 1);          
%         u_CBF = zeros(m, 1);          
%         u_APF = zeros(m, 1);     
% 
%         a = F_att.'*f;
%         b = F_att.'*g;
%         sigma = norm(b)^2;
%         a_tilde = a + sigma;
% 
%         if a_tilde > 0
%             u_CLF = -a_tilde/norm(b)^2 * b.';
%         end
% 
%         alpha = 1;
%         c = F_rep.'*f - alpha;
%         d = F_rep.'*g;
%         gamma = norm(d)^2 + alpha - d*u_CLF;
%         c_tilde = c + gamma;
%         phi = c_tilde + d*u_CLF;
% 
%         %% Standard CBF 
%         if i == 1
%             if phi <= 0
%                 u_CBF = u_CLF;
%             else
%                 u_CBF = u_CLF - phi/norm(d)^2 * d.';
%             end
%         end
% 
%         %% APF variant
%         if i == 2
%             if h >= h_0
%                 u_APF = u_CLF;
%             else
%                 u_APF = u_CLF - phi/norm(d)^2 * d.';
%             end
%         end
%     end
% 
%     %% ODE
%     dxdt(:,1) = A*x(:,1) + B*u_CBF;
%     dxdt(:,2) = A*x(:,2) + B*u_APF;
%     dxdt = reshape(dxdt, [], 1);
% 
% end