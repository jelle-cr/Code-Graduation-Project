function dpdt = odefcn(t,p)      

    load('./Data/Parameters.mat');

    p = reshape(p, n, N_a);
    dpdt = zeros(n, N_a);

    % Current time
    %t     % Printing this each time step approximately doubles required simulation time

    %% Dynamical model
    f = A*p;
    g = B;

    %% Obstacle
    poly = [-1, 1, 1.2, -0.5, -1.5;
            -2, -1, 1,  2, 0];

    %% Controller CLF + CBF safety filter
    H = eye(2);
    F = zeros(m,1);
    options = optimoptions('quadprog', 'Display', 'none'); % Runs approx 2.5 times faster 
        
    u = zeros(m, N_a);
    u_nom = zeros(m, 1);
    for i = 1:N_a 
        p_i = p(1:2,i);
        V = 1/2*K_att*norm(p_i-p_d)^2;
        gradV = K_att*(p_i-p_d);
        LfV = gradV.'*f;
        LgV = gradV.'*g;
        A_CLF = LgV;
        b_CLF = -5*V - LfV;

        u_nom = quadprog(H,F,A_CLF,b_CLF, [],[],[],[],[], options);
        
        [h, A_closest, b_closest] = Functions.distanceToPolytope(p_i, r_a, poly);
        Lfh = A_closest/norm(A_closest)*f;
        Lgh = A_closest/norm(A_closest)*g;

        F = -u_nom;
        A_CBF = -Lgh;
        b_CBF = 10*h + Lfh;

        u(:,i) = quadprog(H,F,A_CBF,b_CBF, [],[],[],[],[], options);

        % Limit control force
        % u(:,i) = min(max(u(:,i), -u_max), u_max);
    end 

    %% ODE
    for i = 1:N_a
        dpdt(:,i) = A*p(:,i) + B*u(:,i);
    end
    dpdt = reshape(dpdt, [], 1);
end