function dxdt = odefcn(t, x)      
    load('./Data/parameters.mat');
    persistent u_att_save u_rep_save

    x = reshape(x, n, N_a);         % Full state matrix
    dxdt = zeros(n, N_a);

    %% Controller
    u = zeros(m, N_a);              % Initialize controller
    u_att = zeros(m, N_a);          
    u_rep = zeros(m, N_a);              
    
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
            u_att(:,i) = -gradU_att;
            u_rep(:,i) = -gradU_rep;
        elseif strcmp(controller, 'APF-SafetyFilter')    % APF with safety filter properties
            F_att = -gradU_att;
            F_rep = -gradU_rep;

            sigma = norm(F_att)^2;
            k_att = sigma/norm(F_att)^2;

            gamma = 0;
            alpha = 1*min(h);
            k_rep = (gamma - alpha - k_att*F_rep.'*F_att)/norm(F_rep)^2;

            u_att(:,i) = k_att*F_att;
            if k_rep > 0
                u_rep(:,i) = k_rep*F_rep;
            end
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
            u_att(:,i) = u_nom;
            
            alpha = 1*min(h);
            c = gradU_rep.'*f - alpha;
            d = gradU_rep.'*g;
            gamma = 0;%alpha - d*u_nom + norm(d)^2;
            c_tilde = c + gamma;
            phi = c_tilde + d*u_nom;

            if phi > 0
                u_rep(:,i) = -phi/norm(d)^2*d.';
            end
        end
        u(:,i) = u_att(:,i) + u_rep(:,i);
    end

    %% ODE
    for i = 1:N_a
        dxdt(:,i) = A*x(:,i) + B*u(:,i);
    end
    dxdt = reshape(dxdt, [], 1);

    %% Save control input (assumes ode4)
    u_att_save(:,:,end+1) = u_att;
    u_rep_save(:,:,end+1) = u_rep;

    u_att = [];
    u_rep = [];
    ind = 1;
    if(t == t_end)
        for i = 1:length(u_att_save)
            if mod(i,4)==2  % Take every 4th sample, starting from 2
                u_att(:,:,ind) = u_att_save(:,:,i);
                u_rep(:,:,ind) = u_rep_save(:,:,i);
                ind = ind + 1;
            end
        end
        save('./Data/SimulationDataRecent.mat', 'u_att', 'u_rep');
    end
end