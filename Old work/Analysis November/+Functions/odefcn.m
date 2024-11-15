function dxdt = odefcn(t,x)      
    load('./Data/Parameters.mat');

    x = reshape(x, n, N_a);         % Full state matrix
    dxdt = zeros(n, 1);

    % Current time
    % t     % Printing this each time step approximately doubles required simulation time

    %% Dynamical model
    p = x(1:2,:,1);
    v = x(3:4,:,1);
    f = [v; zeros(2,1)];
    g = [zeros(2,2); eye(2)];

    %% Controller
    u = zeros(m, N_a);              % Initialize controller
    p_d = x_d(1:2);
    gradU_att = p - p_d;
    p_o = x_o(1:2);
    p_o = [3-1.2*t; 2-1.1*t];
    gradU_rep = Functions.RepulsiveGradient(p, p_o, n, k_rep, rho_0, r_a, r_o, t);

    v_d = -gradU_att - gradU_rep;
    V_cbf = 1/2*(v-v_d).'*(v-v_d);

    u = -2*v-gradU_att-gradU_rep;
    
    % Limit control force
    % u = min(max(u, -u_max), u_max);

    %% ODE
    A = [0, 0, 1, 0;    % State space
         0, 0, 0, 1;
         0, 0, 0, 0;
         0, 0, 0, 0];
    B = [0, 0;
         0, 0;
         1, 0;
         0, 1];
    for i = 1:N_a
        dxdt(:,i) = A*x(:,i) + B*u(:,i);
    end
    dxdt = reshape(dxdt, [], 1);
end