%% Second order (added nominal trajectory)
function dXdt = odefcn(t,X)      

    load('./Data/Parameters.mat');

    X = reshape(X, n, N_a);
    dXdt = zeros(n, N_a);
    
    % Current time
    t

    %% Dynamical model
    f = A*X;
    g = B;

    %% Desired positions
    % X_d = reshape(Functions.calculate_desired_trajectories(t), n, N_a);
    X_d = [3, 3, 3; 
           5, 5, 5];

    %% Controller
    u = zeros(m, N_a);
    u_nom = zeros(m, 1);
    for i = 1:N_a 
        F_att = K_att*(X(:,i)-X_d(:,i));
        
        a = F_att.'*f(:,i);
        b = F_att.'*g;
        a_tilde = a + norm(b)^2;

        if ((a_tilde < 0) || (a_tilde == 0 && norm(b) == 0))
            u_nom = zeros(m, 1);
        end
        if ((a_tilde >= 0) && (norm(b) ~= 0))
            u_nom = -a_tilde/(norm(b)^2)*b.';
        end

        u(:,i) = u_nom;

        rho = zeros(2,1);
        F_rep = zeros(m,1);
        % Compute repulsive force for the agents
        for j = 1:N_a
            if j ~= i
                p_ij = X(:,i) - X(:,j);
                rho(1) = norm(p_ij) - 2*r_a;
                if rho(1) < rho_0 
                    F_rep = F_rep - K_rep/rho(1)^2*(1/rho(1)-1/rho_0)*p_ij/norm(p_ij);
                    if rho(1) < 0             % Agents have collided
                        warning(['Collision between drone ' num2str(i) ' and ' num2str(j) ' at time ' num2str(t)])
                    end
                end
            end
        end

        % Compute repulsive force for obstacles
        if ~isempty(X_o)
            for o = 1:width(X_o)
                p_io = X(:,i) - X_o(:,o);
                rho(2) = norm(p_io) - r_a - r_o;
                if rho(2) < rho_0 
                    F_rep = F_rep - K_rep/rho(2)^2*(1/rho(2)-1/rho_0)*p_ij/norm(p_ij);
                end
                if rho(2) < 0             % Agents have collided
                    warning(['Collision between drone ' num2str(i) ' and obstacle ' num2str(o) ' at time ' num2str(t)])
                end
            end
        else    % No static obstacles present
            rho = rho(1);
        end

        alpha = 1;  % Placeholder, since alpha gets canceled anyways
        c = F_rep.'*f(:,i) - alpha;
        d = F_rep.'*g;
        gamma = norm(F_rep)^2 + alpha   - d*u_nom;
        c_tilde = c + gamma;
        phi = c_tilde + d*u_nom;

        % Control Barrier Function
        if ((phi < 0) || (phi == 0 && norm(d) == 0))  
            u(:,i) = u_nom;
        elseif ((phi >= 0) && (norm(d) ~= 0))
            u(:,i) = u_nom - phi/norm(d)^2*d.';
        end

        % Artificial Potential Field
        % if rho > rho_0  
        %     u(:,i) = u_nom;
        % else 
        %     u(:,i) = u_nom - phi/norm(d)^2*d.';
        % end

        % Limit control force
        % u(:,i) = min(max(u(:,i), -u_max), u_max);
    end 

    %% ODE
    for i = 1:N_a
        dXdt(:,i) = A*X(:,i) + B*u(:,i);
    end
    dXdt = reshape(dXdt, [], 1);
end