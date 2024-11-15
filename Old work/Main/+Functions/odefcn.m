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

        a = F_att.'*f;
        b = F_att.'*g;
        c = F_rep.'*f;
        d = F_rep.'*g;

        if strcmp(controller, 'CLF-CBF-APF')
            if norm(F_rep) > 0
                if norm(g.'*F_rep) == 0
                    warning(['Assumption 1 invalid at t = ', sprintf('%.2f', t)]);
                end
            end
            if norm(b)^2 <= a
                warning(['Assumption 2 invalid at t = ', sprintf('%.2f', t)]);
            end
            if norm(F_att) > 0
                if norm(g.'*F_att) == 0
                    warning(['Assumption 3 invalid at t = ', sprintf('%.2f', t)]);
                end
            end
            u = g.'*(-F_att -F_rep);

        elseif strcmp(controller, 'CBF-APF')
            u_CLF = zeros(m,1);
            sigma = norm(b)^2;
            a_tilde = a + sigma;
            if ((a_tilde >= 0) && (norm(b) ~= 0))
                u_CLF = -a_tilde/norm(b)^2*b.';
            end

            u_CBF = u_CLF;
            gamma = -c -d*u_CLF + norm(d)^2;
            c_tilde = c + gamma;
            phi = c_tilde + d*u_CLF;
            if ((phi >= 0) && (norm(d) ~= 0))
                u_CBF = u_CLF - phi/norm(d)^2*d.';
            end
            u = u_CBF;
        end
    end
    % Limit control force
    u = min(max(u, -u_max), u_max);

    %% ODE
    for i = 1:N_a
        dqdt(:,i) = A*q(:,i) + B*u(:,i);
    end
    dqdt = reshape(dqdt, [], 1);
end