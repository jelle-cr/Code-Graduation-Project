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
    mu = 3;             
    p_d = q_d(1:2);
    v_d = q_d(3:4);
    u = zeros(m, N_a);
    for i = 1:N_a    
        p = q(1:2,i);                  
        v = q(3:4,i);     
        V = 1/2*K_att_p*norm(p-p_d)^2 + 1/2*K_att_v*norm(v-v_d+mu*(p-p_d))^2;
        F_att_p = K_att_p*(p - p_d) + K_att_v*mu*(v - v_d + mu*(p - p_d));
        F_att_v = K_att_v*(v - v_d + mu*(p - p_d));
        F_rep_p = zeros(m,1);
        F_rep_v = zeros(m,1);
        % for j = 1:N_a       % Add the repulsive force of all agents
        %     if i ~= j
        %         grad_U_rep = Functions.RepulsiveGradient(q(:,i), q(:,j), n, K_rep, rho_0, r_a, r_a, u_max); %q_i, q_j, K_rep, rho_0, r_a, r_o, a_max
        %         F_rep_p = F_rep_p + grad_U_rep(1:2);
        %         F_rep_v = F_rep_v + grad_U_rep(3:4);
        %     end
        % end
        % [~, o] = min(sum((q_o(1:2,:) - p(:,i)).^2, 1));     % Find closest obstacle index
        for o = 1:N_o       % Add the repulsive force of all obstacles
            [grad_U_rep, rho, rho_m] = Functions.RepulsiveGradient(q(:,i), q_o(:,o), n, K_rep, rho_0, r_a, r_o, u_max); %q_i, q_j, K_rep, rho_0, r_a, r_o, a_max
            F_rep_p = F_rep_p + grad_U_rep(1:2);
            F_rep_v = F_rep_v + grad_U_rep(3:4);
        end

        if strcmp(controller, 'APF')
            F_att = F_att_p + F_att_v;
            F_rep = F_rep_p + F_rep_v;
            u(:,i) = -F_att - F_rep;
        else
            u_CLF = zeros(m, 1);
            u_CBF = zeros(m, 1);
            F_att = [F_att_p;
                     F_att_v];
            F_rep = [F_rep_p;
                     F_rep_v];

            % Control Lyapunov Function
            a = F_att.'*f(:,i);
            b = F_att.'*g;
            sigma = norm(b)^2;%5*V;%
            a_tilde = a + sigma;

            if ((a_tilde >= 0) && (norm(b) ~= 0))
                u_CLF = -a_tilde/norm(b)^2*b.';
            end
            u(:,i) = u_CLF;     % Initialization
    
            % Control Barrier Function
            c = F_rep.'*f(:,i);
            d = F_rep.'*g;

            h = rho-rho_m;
            gamma = -h;%norm(d)^2;%-h;
            c_tilde = c + gamma;
            phi = c_tilde;
            u_CBF = u_CLF - phi/norm(d)^2*d.';
            
            % if phi < 0  % if true S_AC-1 is nonempty
            %     t
            % end
            if strcmp(controller, 'CLF-CBF')
                if ((phi < 0) || (phi == 0 && norm(d) == 0))  
                    u(:,i) = u_CLF;
                elseif ((phi >= 0) && (norm(d) ~= 0))
                    u(:,i) = u_CBF;
                end
            elseif strcmp(controller, 'semi-APF')
                if norm(F_rep) == 0 % only true if rho-rho_m > rho_0
                    u(:,i) = u_CLF;
                else
                    u(:,i) = u_CBF;
                end        
            end

            % H = eye(2);
            % f = -u_CLF;
            % options = optimoptions('quadprog', 'Display', 'none'); % Runs approx 2.5 times faster 
            % u(:,i) = quadprog(H,f,d,-c_tilde, [],[],[],[],[], options);

            H = eye(2);
            f = -u_CLF;
            [A_closest, b_closest] = distanceToPolytope(p,r_a);;
            A_CBF = -A_closest;
            b_CBF = 1*A_closest*v+1*(A_closest*p+b_closest);
            options = optimoptions('quadprog', 'Display', 'none'); % Runs approx 2.5 times faster 
            u(:,i) = quadprog(H,f,A_CBF,b_CBF, [],[],[],[],[], options);

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