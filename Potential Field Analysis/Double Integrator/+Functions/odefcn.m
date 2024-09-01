function dqdt = odefcn(t,q)      

    load('./Data/Parameters.mat');

    q = reshape(q, n, N_a);
    dqdt = zeros(n, N_a);

    % Current time
    % t     % Printing this each time step approximately doubles required simulation time

    %% Dynamical model
    f = A*q;
    g = B;

    %% Controller APF
    u = zeros(m, N_a);
    u_nom = zeros(m, 1);
    for i = 1:N_a 
        F_att = -K_att_p*(q(1:2,i)-q_d(1:2)) -K_att_v*(q(3:4,i)-q_d(3:4));
        F_rep = zeros(m,1);
        for o = 1:N_o
            p_io = q(1:2,i) - q_o(1:2,o);
            v_io = q(3:4,i) - q_o(3:4,o);
            p_hat = -p_io/norm(p_io);
            v_r = v_io.'*p_hat;
            rho = norm(p_io) - r_a - r_o;   
            rho_m = v_r^2/(2*u_max);
            if rho - rho_m < rho_0
                if v_r > 0
                    F_rep_base = K_rep/(rho-rho_m)^2*(1/(rho-rho_m)-1/rho_0);
                    F_rep_p = F_rep_base*(v_r/u_max*((p_io*p_io.'*v_io)/norm(p_io)^3 - v_io/norm(p_io)) - p_hat);
                    F_rep_v = F_rep_base*(v_r/u_max*p_hat);
                    if rho - rho_m >= 0
                        F_rep = F_rep + F_rep_p + F_rep_v;
                    else   % undefined      
                        warning('undefined')
                        F_rep = F_rep - F_rep_p - F_rep_v;    % Collision guaranteed (Also flip sign of repulsive force)
                    end
                end
            end
        end
        u(:,i) = F_att + F_rep;

        % Limit control force
        u(:,i) = min(max(u(:,i), -u_max), u_max);
    end 

    %% Controller CBF
    % u = zeros(m, N_a);
    % u_nom = zeros(m, 1);
    % for i = 1:N_a 
    %     F_att = K_att*(p(1:2,i)-p_d);
    %     F_rep = zeros(m,1);
    % 
    %     a = F_att.'*f(1:2,i);
    %     b = F_att.'*g;
    %     a_tilde = a + norm(b)^2;
    % 
    %     if ((a_tilde < 0) || (a_tilde == 0 && norm(b) == 0))
    %         u_nom = zeros(m, 1);
    %     end
    %     if ((a_tilde >= 0) && (norm(b) ~= 0))
    %         u_nom = -a_tilde/(norm(b)^2)*b.';
    %     end
    % 
    %     u(:,i) = u_nom;
    % 
    %     rho = inf(N_o,1);   % Initialize all agent and obstacle distances to inf
    %     F_rep = zeros(m,1);
    % 
    % 
    %     % Compute repulsive force for obstacles
    %     if N_o > 0
    %         for o = 1:N_o
    %             p_io = p(1:2,i) - p_o(1:2,o);
    %             rho(o) = norm(p_io) - r_a - r_o;
    %             if rho(o) < rho_0 
    %                 if rho(o) >= 0 
    %                     F_rep = F_rep - K_rep/rho(o)^2*(1/rho(o)-1/rho_0)*p_io/norm(p_io);
    %                 else       % Agent and obstacle have collided
    %                     F_rep = F_rep + K_rep/rho(o)^2*(1/rho(o)-1/rho_0)*p_io/norm(p_io);
    %                 end
    %             end
    %         end
    %     end
    % 
    %     alpha = 1;  % Placeholder, since alpha gets canceled anyways
    %     c = F_rep.'*f(1:2,i) - alpha;
    %     d = F_rep.'*g;
    %     gamma = norm(F_rep)^2 + alpha   - d*u_nom;
    %     c_tilde = c + gamma;
    %     phi = c_tilde + d*u_nom;
    % 
    %     % Control Barrier Function
    %         if ((phi < 0) || (phi == 0 && norm(d) == 0))  
    %             u(:,i) = u_nom;
    %         elseif ((phi >= 0) && (norm(d) ~= 0))
    %             u(:,i) = u_nom - phi/norm(d)^2*d.';
    %         end
    % end

    %% ODE
    for i = 1:N_a
        dqdt(:,i) = A*q(:,i) + B*u(:,i);
    end
    dqdt = reshape(dqdt, [], 1);
end