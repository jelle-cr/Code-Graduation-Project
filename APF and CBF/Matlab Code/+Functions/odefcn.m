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
    X_d = [3, 3; 
           5, 5];

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
        % Compute repulsive force for closest agent
        agentDistances = sum((X - X(:, i)).^2, 1);
        agentDistances(i) = Inf;    % Set self to infinite 
        [~, closestAgent] = min(agentDistances);
        if closestAgent ~= i
            p_ij = X(:,i) - X(:,closestAgent);
            rho(1) = norm(p_ij) - 2*r_a;

            if rho(1) < rho_0 
               F_rep = F_rep - K_rep/rho(1)^2*(1/rho(1)-1/rho_0)*p_ij/norm(p_ij);
            end
            if rho(1) < 0             % Agents have collided
               warning(['Collision between drone ' num2str(i) ' and ' num2str(closestAgent) ' at time ' num2str(t)])
            end
        end

        % Compute repulsive force for closest obstacle
        if ~isempty(X_o)
            [~, closestObstacle] = min(sum((X(:,i)-X_o).^2, 1));
            p_io = X(:,i) - X_o(:,closestObstacle);
            rho(2) = norm(p_io) - r_a - r_o;
            if rho(2) < rho_0 
                F_rep = F_rep - K_rep/rho(2)^2*(1/rho(2)-1/rho_0)*p_ij/norm(p_ij);
            end
            if rho(2) < 0             % Agents have collided
                warning(['Collision between drone ' num2str(i) ' and obstacle ' num2str(closestObstacle) ' at time ' num2str(t)])
            end
        else
            rho(2) = rho(1);
        end

        alpha = 1;
        c = F_rep.'*f(:,i) - alpha;
        d = F_rep.'*g;
        gamma = norm(F_rep)^2 + alpha   - d*u_nom;
        c_tilde = c + gamma;
        phi = c_tilde + d*u_nom;

        % if ((phi < 0) || (phi == 0 && norm(d) == 0))  % CBF
        %     u = u_nom;
        % elseif ((phi >= 0) && (norm(d) ~= 0))
        %     u = u_nom - phi/norm(d)^2*d.';
        %     % u = - phi/norm(d)^2*d.';
        % end
        % rho

        if rho > rho_0  % APF
            u(:,i) = u_nom;
        else 
            u(:,i) = u_nom - phi/norm(d)^2*d.';
        end
        u(:,i) = min(max(u(:,i), -u_max), u_max);
    end 


    if max(max(u)) > 10
        u
        t
    end

    %% ODE
    % dXdt(1,:) = X(2) + u(1);
    % dXdt(2,:) = X(1) + u(2);
    dXdt(1,:) = u(1,:);
    dXdt(2,:) = u(2,:);
    dXdt = reshape(dXdt, [], 1);
end