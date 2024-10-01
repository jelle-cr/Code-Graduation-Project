function dqdt = odefcn(t,q)      
    load('./Data/Parameters.mat');

    q = reshape(q, n, N_a);         % Full state matrix
    dqdt = zeros(n, 1);

    % Current time
    % t     % Printing this each time step approximately doubles required simulation time

    %% Dynamical model
    f = A*q;
    g = B;

    %% Controller
    u = zeros(m, N_a);              % Initialize controller
    e_q = q-q_d;                    % State error
    % U_att = 1/2*e_q.'*W_att*e_q;    % Lyapunov function
    F_att = W_att*e_q;              % Gradient of Lyapunov function

    [F_rep, h] = Functions.RepulsiveGradient(q, q_o, n, k_rep, rho_0, r_a, r_o, u_max);

    if strcmp(controller, 'APF')
        if n == 2       %Single integrator
            u = -F_att - F_rep;
        elseif n == 4   %Double integrator
            u = -F_att(1:2) - F_att(3:4) - F_rep(1:2) - F_rep(3:4);
        end
    else
        % u_CLF = zeros(m, 1);
        % u_CBF = zeros(m, 1);

        % a = F_att.'*f;
        % b = F_att.'*g;
        % c = F_rep.'*f;
        % d = F_rep.'*g;
        
        u = g.'*(-F_att -F_rep);
    end
    % Limit control force
    u = min(max(u, -u_max), u_max);

    %% ODE
    for i = 1:N_a
        dqdt(:,i) = A*q(:,i) + B*u(:,i);
    end
    dqdt = reshape(dqdt, [], 1);
end