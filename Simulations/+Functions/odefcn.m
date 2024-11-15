function dxdt = odefcn(t, x)      
    load('./Data/parameters.mat');
    persistent u_save

    x = reshape(x, n, N_a);         % Full state matrix
    dxdt = zeros(n, N_a);

    %% Controller
    u = zeros(m, N_a);              % Initialize controller
    
    p = x;
    p_d = x_d;
    p_o = x_o;
    for i = 1:N_a
        gradU_att = zeros(m,1);
        gradU_rep = zeros(m,1);

        gradU_att = k_att*(p(:,i)-p_d);
        
        for j = 1:N_o
            p_ij = p(:,i)-p_o(:,j);

            h = norm(p_ij) - r_a - r_o;

            if h < rho_0
                gradU_rep = gradU_rep - k_rep/h^2*(1/h-1/rho_0)*p_ij/norm(p_ij);
            end
        end
        u(:,i) = -gradU_att - gradU_rep;
    end

    %% ODE
    for i = 1:N_a
        dxdt(:,i) = A*x(:,i) + B*u(:,i);
    end
    dxdt = reshape(dxdt, [], 1);

    %% Save control input (assumes ode4)
    u_save(:,:,end+1) = u;

    u = [];
    ind = 1;
    if(t == t_end)
        for i = 1:length(u_save)
            if mod(i,4)==2  % Take every 4th sample, starting from 2
                u(:,:,ind) = u_save(:,:,i);
                ind = ind + 1;
            end
        end
        save('./Data/SimulationData.mat', 'u');
    end
end