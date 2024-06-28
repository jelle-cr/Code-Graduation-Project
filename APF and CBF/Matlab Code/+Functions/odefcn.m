%% Second order (added nominal trajectory)
function dXdt = odefcn(t,X)      
    global u_save

    % load('./Data/Parameters.mat');

    n = 2;
    m = 2;
    rho_0 = 0.1;
    N_a = 2;
    K_att = 10;
    K_rep = 0.001;
    r_a = 0.5;
    u_max = 10;

    % X = reshape(X, n, N_a);
    dXdt = zeros(n, 1);
    % time
    t

    %% Dynamic model
    % f = [X(3,:); 
    %      X(4,:);
    %      -d/m*X(3,:);
    %      -d/m*X(4,:)];
    % g = [0, 0;
    %      0, 0;
    %      1/M, 0;
    %      0, 1/M];
    %% Kinematic model
    f = [X(2); X(1)];
    g = [1, 0;
         0, 1];

    %% CLF nominal controller
    % Nominal trajectories
    % X_d = reshape(Functions.calculate_desired_trajectories(t), n, N_a);
    % X_d = [3, 0; 5, 0];
    X_d = [0;0];
    X_o = [1;1.5];
    u = zeros(m, N_a);
    
    for i = 1:1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%N_a   
        F_att = K_att*(X-X_d);
        F_rep = zeros(m,1);
        for j = 1:N_a
            if i ~= j
                p_ij = X - X_o;
                rho = norm(p_ij) - 2*r_a;
                if rho < rho_0 
                    F_rep = F_rep - K_rep/rho^2*(1/rho-1/rho_0)*p_ij/norm(p_ij);
                end
                if rho < 0             % Agents have collided
                    warning(['Collision between drone ' num2str(i) ' and ' num2str(j) ' at time ' num2str(t)])
                end
            end
        end
        a = F_att.'*f;
        b = F_att.'*g;
        a_tilde = a + norm(b)^2;

        if (a_tilde < 0)
            u_nom = zeros(m, 1);
        end
        if (a_tilde == 0 && norm(b) == 0)
            u_nom = zeros(m, 1);
        end
        if ((a_tilde >= 0) && (norm(b) ~= 0))
            u_nom = -a_tilde/(norm(b)^2)*b.';
        end

        u = u_nom;

        alpha = 1;
        c = F_rep.'*f - alpha;
        d = F_rep.'*g;
        gamma = norm(F_rep)^2 + alpha;%%%%%%%%%%%%% - d*u_nom;
        c_tilde = c + gamma;
        phi = c_tilde + d*u_nom;

        % if ((phi < 0) || (phi == 0 && norm(d) == 0))
        %     u(:,i) = u_nom;
        % elseif ((phi >= 0) && (norm(d) ~= 0))
        %     u(:,i) = u_nom - phi/norm(d)^2*d.';
        % 
        % end
        if rho > rho_0
            u = u_nom;
        else 
            % u(:,i) = u_nom - phi/norm(d)^2*d.';
            u =  - phi/(norm(d)^2)*d.';%%%%%%%%%%%%%%%%%%%%%
        end
        % u(:,i) = min(max(u(:,i), -u_max), u_max);
    end    

    u_save=[u_save, u];

    %% ODE
    % dXdt(1,:) = 1/M * u(1,:); 
    % dXdt(2,:) = 1/M * u(2,:); 
    % dXdt(1,:) = X(3,:);                           
    % dXdt(2,:) = X(4,:);
    % dXdt(3,:) = -d/m*X(3,:) + 1/m * u(1,:); %+ r_a*(2*(rand(1,2)-0.5)); %Simulate some noise force
    % dXdt(4,:) = -d/m*X(4,:) + 1/m * u(2,:); %+ r_a*(2*(rand(1,2)-0.5));
    dXdt(1,:) = X(2) + u(1);
    dXdt(2,:) = X(1) + u(2);
    % dXdt(:,2) = [0;0];  % agent 2 in standstill
    % dXdt = reshape(dXdt, [], 1);
end