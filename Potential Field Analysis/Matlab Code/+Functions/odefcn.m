function dpdt = odefcn(t,p)      

    load('./Data/Parameters.mat');

    p = reshape(p, n, N_a);
    dpdt = zeros(n, N_a);

    % Current time
    % t     % Printing this each time step approximately doubles required simulation time

    %% Dynamical model
    f = A*p;
    g = B;

    %% Controller
    u = zeros(m, N_a);
    u_nom = zeros(m, 1);
    for i = 1:N_a 
        F_att = K_att*(p(1:2,i)-p_d);
        F_rep = zeros(m,1);
        for o = 1:N_o
            p_io = p(1:2,i) - p_o(:,o);
            rho = norm(p_io) - r_a - r_o;   
            if rho < rho_0
                if rho >= 0
                    F_rep = F_rep - K_rep/rho^2*(1/rho-1/rho_0)*p_io/norm(p_io);
                else            
                    F_rep = F_rep + K_rep/rho^2*(1/rho-1/rho_0)*p_io/norm(p_io);    % Collision (Also flip sign of repulsive force)
                end
            end
        end
        u(:,i) = -F_att - F_rep;
        
        % Limit control force
        u(:,i) = min(max(u(:,i), -u_max), u_max);
    end 

    %% ODE
    for i = 1:N_a
        dpdt(:,i) = A*p(:,i) + B*u(:,i);
    end
    dpdt = reshape(dpdt, [], 1);
end