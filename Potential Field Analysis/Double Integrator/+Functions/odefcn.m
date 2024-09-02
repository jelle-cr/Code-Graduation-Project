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
        F_att_p = -K_att_p*(p(:,i)-q_d(1:2));
        F_att_v = -K_att_v*(v(:,i)-q_d(3:4));
        F_rep_p = zeros(m,1);
        F_rep_v = zeros(m,1);
        for o = 1:N_o       % Add the repulsive force of all obstacles
            grad_U_rep = Functions.RepulsiveGradient(q(:,i), q_o(:,o), n, K_rep, rho_0, r_a, r_o, u_max); %q_i, q_j, K_rep, rho_0, r_a, r_o, a_max
            F_rep_p = F_rep_p - grad_U_rep(1:2);
            F_rep_v = F_rep_v - grad_U_rep(3:4);
        end
        F_att = F_att_p + F_att_v;
        F_rep = F_rep_p + F_rep_v;

        u(:,i) = F_att + F_rep;

        % Limit control force
        u(:,i) = min(max(u(:,i), -u_max), u_max);
    end 


    %% ODE
    for i = 1:N_a
        dqdt(:,i) = A*q(:,i) + B*u(:,i);
    end
    dqdt = reshape(dqdt, [], 1);
end