function dxdt = odefcn(t, x)      
    load('./Data/parameters.mat');
    persistent u_att_save u_rep_save 

    x = reshape(x, n, N_a);         % Full state matrix
    dxdt = zeros(n, N_a);

    if mod(t,1)==0                  % Display time at certain intervals
        t
    end

    %% Controller                   % Either uses APF or APF-safety filter
    u = zeros(m, N_a);              % Initialize controller
    u_att = zeros(m, N_a);          
    u_rep = zeros(m, N_a);       
    
    p = x(1:2,:);
    p_d = x_d(1:2);
    p_o = x_o(1:2,:);
    for i = 1:N_a
        [gradU_att, gradU_rep, h] = Functions.potential_gradients(m, p(:,i), p_d, p_o, k_att, k_rep, r_a, r_o, h_0);
        F_att = -gradU_att;
        F_rep = -gradU_rep;
        if strcmp(controller, 'APF')
            u_att(:,i) = F_att;
            u_rep(:,i) = F_rep;
        elseif strcmp(controller, 'APF-SF')    % APF with safety filter properties
            sigma = k_sigma*norm(F_att)^2;
            gamma = k_gamma*norm(F_rep)^2;
            alpha = k_alpha*min(h);
            [u_att(:,i), u_rep(:,i), ~] = Functions.APF_safety_filter(m, F_att, F_rep, sigma, gamma, alpha);
        end
        u(:,i) = u_att(:,i) + u_rep(:,i);
    end

    if strcmp(dynamics, 'Double Integrator')    % Proportional velocity feedback controller
        v_d = u;
        v = x(3:4,:);
        e_v = v - v_d;
        u = -k_pid*e_v;
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