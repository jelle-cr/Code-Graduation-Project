function dxdt = odefcn(t, x)      
    load('./Data/parameters.mat');

    x_full = reshape(x, n, N_a);
    dxdt = zeros(n, 1);

    for i = 1:N_a   % Simulate all trajectories for this time-step 1-by-1
        x = squeeze(x_full(:,i));

        %% Dynamics
        f = A*x(:);
        g = B;
    
        %% Potential fields
        p = x(1:2, :);              % Current position
        p_d = x_d(1:2);             % Desired position
        p_o = x_o(1:2, :);          % Obstacle positions
    
        % Calculate the gradients of the potential fields, as well as the distances to obstacles
        [F_att, F_rep, h] = Functions.potential_gradients(m, p, p_d, p_o, k_att, k_rep, r_a, r_o, rho_0);
    
        %% Controller               
        u = zeros(m, 1);            % Initialize controller
        u_CLF = zeros(m, 1);          
        u_CBF = zeros(m, 1);          
        u_APF = zeros(m, 1);     
    
        a = F_att.'*f;
        b = F_att.'*g;
        sigma = norm(b)^2;          % CLF tightening parameter
        a_tilde = a + sigma;
    
        if a_tilde > 0
            u_CLF = -a_tilde/norm(b)^2 * b.';
        end
    
        alpha = 1;
        c = F_rep.'*f - alpha;
        d = F_rep.'*g;
        gamma = norm(d)^2 + alpha - d*u_CLF;    % CBF tightening parameter
        c_tilde = c + gamma;
        phi = c_tilde + d*u_CLF;
    
        if i == 1   % Nominal controller
            u = u_CLF;
        end

        if i == 2   % AC controller
            if phi <= 0
                u_CBF = u_CLF;
            else
                u_CBF = u_CLF - phi/norm(d)^2 * d.';
            end
            u = u_CBF;
        end
        
        if i == 3   % APF-AC controller
            if min(h) >= rho_0
                u_APF = u_CLF;
            else
                u_APF = u_CLF - phi/norm(d)^2 * d.';
            end
            u = u_APF;
        end

        %% ODE
        dxdt(:,i) = A*x(:) + B*u;
    end
    dxdt = reshape(dxdt, [], 1);
end