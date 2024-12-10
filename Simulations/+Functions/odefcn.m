function dxdt = odefcn(t, x)      
    load('./Data/parameters.mat');
    persistent u_att_save u_rep_save gamma_min norm_e_vprev

    if t == 0
        gamma_min = 0;
        norm_e_vprev = 0;
    end

    x = reshape(x, n, N_a);         % Full state matrix
    dxdt = zeros(n, N_a);

    if mod(t,1)==0               % Display time at certain intervals
        t
    end

    %% Controller
    u = zeros(m, N_a);              % Initialize controller
    u_att = zeros(m, N_a);          
    u_rep = zeros(m, N_a);       
    
    p = x(1:2,:);
    p_d = x_d(1:2);
    p_o = x_o(1:2,:);
    for i = 1:N_a
        [gradU_att, gradU_rep, h] = Functions.potential_gradients(m, p(:,i), p_d, p_o, k_att, k_rep, r_a, r_o, rho_0);
        F_att = -gradU_att;
        F_rep = -gradU_rep;
        if strcmp(controller, 'APF')
            u_att(:,i) = F_att;
            u_rep(:,i) = F_rep;
        elseif strcmp(controller, 'SF')    % APF with safety filter properties
            sigma = norm(F_att)^2;
            gamma = k_gamma*norm(F_rep)^2;
            % gamma = 0;
            alpha = k_alpha*min(h);
            [u_att(:,i), u_rep(:,i)] = Functions.APF_safety_filter(m, F_att, F_rep, sigma, gamma, alpha);
        elseif strcmp(controller, 'CBF')    % CBF-QP + CLF
            % u_nom = zeros(m,1);
            % f = A*x(:,i);
            % g = B;
            % 
            % a = gradU_att.'*f;
            % b = gradU_att.'*g;
            % sigma = norm(b)^2;
            % a_tilde = a + sigma;
            % 
            % if a_tilde > 0
            %     u_nom = -a_tilde/norm(b)^2*b.';
            % end
            % u_att(:,i) = u_nom;
            % 
            % alpha = 1*min(h);
            % c = gradU_rep.'*f - alpha;
            % d = gradU_rep.'*g;
            % % gamma = norm(d)^2;
            % gamma = 0;%alpha - d*u_nom + norm(d)^2;
            % c_tilde = c + gamma;
            % phi = c_tilde + d*u_nom;
            % 
            % if phi > 0
            %     u_rep(:,i) = -phi/norm(d)^2*d.';
            % end
        end
        u(:,i) = u_att(:,i) + u_rep(:,i);
    end

    if strcmp(dynamics, 'Double Integrator')    % Proportional velocity feedback controller
        v_d = u;
        v = x(3:4,:);
        e_v = v_d-v;
        u = k_pid*e_v;

        norm_e_vprev = norm(e_v);
        
        % t
        % desired = gamma - alpha - F_rep.'*v_d
        % actual = gamma - alpha - F_rep.'*v
        % gamma
        % gamma_min = max(0,F_rep.'*e_v)
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