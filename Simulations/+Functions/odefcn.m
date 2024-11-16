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
    
        % Attractive gradient
        gradU_att = k_att*(p(:,i)-p_d);
           
        % Repulsive gradient
        for j = 1:N_o
            p_ij = p(:,i)-p_o(:,j);
            h(j) = norm(p_ij) - r_a - r_o;
            if h(j) < rho_0
                gradU_rep = gradU_rep - k_rep/h(j)^2*(1/h(j)-1/rho_0)*p_ij/norm(p_ij);
            end
        end

        if strcmp(controller, 'APF')
            u(:,i) = -gradU_att - gradU_rep;
        elseif strcmp(controller, 'CBF')    % CBF-QP + CLF
            u_nom = zeros(m,1);
            f = A*x(:,i);
            g = B;
            
            a = gradU_att.'*f;
            b = gradU_att.'*g;
            sigma = norm(b)^2;
            a_tilde = a + sigma;

            if a_tilde > 0
                u_nom = -a_tilde/norm(b)^2*b.';
            end
            
            alpha = min(h);
            c = gradU_rep.'*f - alpha;
            d = gradU_rep.'*g;
            gamma = 0;%alpha - d*u_nom + norm(d)^2;
            c_tilde = c + gamma;
            phi = c_tilde + d*u_nom;

            if phi <= 0
                u(:,i) = u_nom;
            else
                u(:,i) = u_nom - phi/norm(d)^2*d.';
            end
        end
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
        save('./Data/SimulationDataRecent.mat', 'u');
    end
end