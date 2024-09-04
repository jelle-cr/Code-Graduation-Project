function dqdt = odefcn(t,q)      
    load('./Data/Parameters.mat');

    q = reshape(q, n, N_a);         % Full state matrix
    dqdt = zeros(n, N_a);

    p = q(1:2,:);                   % Position state matrix
    v = q(3:4,:);                   % Velocity state matrix

    % Current time
    % t     % Printing this each time step approximately doubles required simulation time

    %% Dynamical model
    f = A*q;
    g = B;

    %% Controller APF
    u = zeros(m, N_a);
    for i = 1:N_a
        F_att_p = K_att_p*(p(:,i)-q_d(1:2));
        F_att_v = K_att_v*(v(:,i)-q_d(3:4));
        F_rep_p = zeros(m,1);
        F_rep_v = zeros(m,1);
        for o = 1:N_o       % Add the repulsive force of all obstacles
            grad_U_rep = Functions.RepulsiveGradient(q(:,i), q_o(:,o), n, K_rep, rho_0, r_a, r_o, u_max); %q_i, q_j, K_rep, rho_0, r_a, r_o, a_max
            F_rep_p = F_rep_p + grad_U_rep(1:2);
            F_rep_v = F_rep_v + grad_U_rep(3:4);
        end

        if strcmp(controller, 'APF')
            F_att = F_att_p + F_att_v;
            F_rep = F_rep_p + F_rep_v;
            u(:,i) = -F_att - F_rep;
        else
            u_CLF = zeros(m, 1);
            F_att = -[F_att_p;
                     F_att_v];
            F_rep = -[F_rep_p;
                     F_rep_v];

            % Control Lyapunov Function
            a = F_att.'*f(:,i);
            b = F_att.'*g;
            sigma = norm(b)^2;
            a_tilde = a + sigma;

            % H = eye(2);
            % f = zeros(2,1);
            % u(:,i) = quadprog(H,f,b,-a_tilde);
            if ((a_tilde >= 0) && (sigma ~= 0))
                u_CLF = -a_tilde/sigma*b.';
            end
            % Control Barrier Function
            c = F_rep.'*f(:,i);
            d = F_rep.'*g;
            gamma = norm(d)^2;
            c_tilde = c + gamma;
            phi = c_tilde;
            if ((phi < 0) || (phi == 0 && gamma == 0))  
                u(:,i) = u_CLF;
            elseif ((phi >= 0) && (gamma ~= 0))
                u(:,i) = u_CLF - phi/gamma*d.';
            end
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