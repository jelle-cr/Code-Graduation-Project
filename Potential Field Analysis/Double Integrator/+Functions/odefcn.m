function dqdt = odefcn(t,q)      
    load('./Data/Parameters.mat');

    q = reshape(q, n, N_a);         % Full state matrix
    dqdt = zeros(n, N_a);

    % Current time
    % t     % Printing this each time step approximately doubles required simulation time

    %% Dynamical model
    f = A*q;
    g = B;

    %% Controller APF
                 
    p_d = q_d(1:2);
    v_d = q_d(3:4);
    u = zeros(m, N_a);
    for i = 1:N_a    
        p = q(1:2,i);                  
        v = q(3:4,i);  
        e_q = [p-p_d; v-v_d];       % State error
        V = 1/2*e_q.'*W_att*e_q;    % Lyapunov function
        F_att = W_att*e_q;          % Gradient of Lyapunov function
        F_rep_p = zeros(m,1);
        F_rep_v = zeros(m,1);

        % for j = 1:N_a       % Add the repulsive force of all agents
        %     if i ~= j
        %         grad_U_rep = Functions.RepulsiveGradient(q(:,i), q(:,j), n, k_rep, rho_0, r_a, r_a, u_max); %q_i, q_j, k_rep, rho_0, r_a, r_o, a_max
        %         F_rep_p = F_rep_p + grad_U_rep(1:2);
        %         F_rep_v = F_rep_v + grad_U_rep(3:4);
        %     end
        % end
        for o = 1:N_o       % Add the repulsive force of all obstacles
            [grad_U_rep, rho, rho_m] = Functions.RepulsiveGradient(q(:,i), q_o(:,o), n, k_rep, rho_0, r_a, r_o, u_max); %q_i, q_j, k_rep, rho_0, r_a, r_o, a_max
            F_rep_p = F_rep_p + grad_U_rep(1:2);
            F_rep_v = F_rep_v + grad_U_rep(3:4);
        end

        if strcmp(controller, 'APF')
            F_att = F_att(1:2) + F_att(3:4);
            F_rep = F_rep_p + F_rep_v;
            u(:,i) = -F_att - F_rep;
        else
            u_CLF = zeros(m, 1);
            u_CBF = zeros(m, 1);
            F_rep = [F_rep_p;
                     F_rep_v];

            % Control Lyapunov Inequality
            a = F_att.'*f(:,i);
            b = F_att.'*g;
            sigma = 10*V;
            % sigma = norm(b)^2
            a_tilde = a + sigma;

            if ((a_tilde >= 0) && (norm(b) ~= 0))
                u_CLF = -a_tilde/norm(b)^2*b.';
            end

            u(:,i) = u_CLF;     % Initialization
    
            % Control Barrier Inequality
            c = F_rep.'*f(:,i);
            d = F_rep.'*g;

            h = rho-rho_m;
            gamma = -5*h;%norm(d)^2;
            c_tilde = c + gamma;
            c_tilde = 10^1*norm(d)^2;
            phi = c_tilde + d*u_CLF;
            % phi = norm(d)^2;

           %  H = eye(2);
           %  F = -u_CLF;
           %  options = optimoptions('quadprog', 'Display', 'none'); % Runs approx 2.5 times faster 
           %  if norm(F_rep) ~= 0
           %      % u(:,i) = quadprog(H,F,d,-c_tilde, [],[],[],[],[], options); 
           %      u(:,i) = quadprog(H,F,d,-c_tilde, [],[],[],[],[], options); 
           % end

            % if phi < 0  % if true S_AC-1 is nonempty
            %     t
            % end
            if strcmp(controller, 'CLF-CBF')
                if (phi < 0) || (phi == 0 && norm(d) == 0)
                    u_CBF = u_CLF;
                elseif ((phi >= 0) && (norm(d) ~= 0))
                    u_CBF = u_CLF - phi/norm(d)^2*d.';
                end
            elseif strcmp(controller, 'APF-CBF')
                if norm(F_rep) == 0 % only true if rho-rho_m > rho_0
                    u_CBF = u_CLF;
                else
                    u_CBF = u_CLF - phi/norm(d)^2*d.';
                end        
            end
            u(:,i) = u_CBF;
        end
        % Limit control force
        u(:,i) = min(max(u(:,i), -u_max), u_max);
    end 


    %% ODE
    for i = 1:N_a
        dqdt(:,i) = A*q(:,i) + B*u(:,i);
    end
    dqdt = reshape(dqdt, [], 1);
end