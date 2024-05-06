%% Second order (added nominal trajectory)
function dpdt = odefcn(t,p)      
    global u_save u_nom_save

    load('./Data/Parameters.mat');

    p = reshape(p, states, N_a);
    dpdt = zeros(states, N_a);
    % Note that [p(1) p(2) p(3) p(4)]==[x y x' y']
    % time
    t

    %% Dynamics
    f = [p(3); p(4); -d/m*p(3); -d/m*p(4)];
    g = [0 0; 0 0; 1/m 0; 0 1/m];

    %% CLF nominal controller
    % Nominal trajectories
    p_nom = Functions.calculate_nominal_trajectories(t);
    u_nom = zeros(dimensions,N_a);

    H = 2*eye(dimensions);
    F = zeros(1,dimensions);
    for i = 1:N_a
        A = [];
        b = [];
        V = 1/2*(p(1,i)-p_nom(1,i))^2 + 1/2*(p(2,i)-p_nom(2,i))^2 + 1/2*(p(3,i)+l2*(p(1,i)-p_nom(1,i)))^2 + 1/2*(p(4,i)+l3*(p(2,i)-p_nom(2,i)))^2;
        gradV = [p(1,i) - p_nom(1,i) + l2*(p(3,i)-p_nom(3,i)+l2*(p(1,i)-p_nom(1,i)));
                 p(2,i) - p_nom(2,i) + l3*(p(4,i)-p_nom(4,i)+l3*(p(2,i)-p_nom(2,i)));
                 p(3,i) - p_nom(3,i) + l2*(p(1,i)-p_nom(1,i));
                 p(4,i) - p_nom(4,i) + l3*(p(2,i)-p_nom(2,i))];
        L_fV= dot(gradV, f);
        L_gV= [dot(gradV, g(:,1)), dot(gradV, g(:,2))];
        A = L_gV;
        b = -L_fV-lambda*V;
        % Add input constraints, to prevent u_nom from exploding
        A = [A; [1, 0; 0, -1]];
        b = [b; u_max; u_max];

        u_nom(:,i) = quadprog(H, F, A, b);
    end

    %% CBF Safety filter
    u = zeros(dimensions, N_a);
    H = 2*eye(dimensions);
    for i = 1:N_a
        A = [];
        b = [];
        for j = 1:N_a
            if i ~= j
                xi_ij = p(1 : dimensions, i) - p(1 : dimensions, j);
                % if norm(xi_ij) < barrierFunctionMaxDistance
                    v_ij = p(dimensions+1 : 2*dimensions, i) - p(dimensions+1 : 2*dimensions, j);
                    h = xi_ij.'*xi_ij - (2*barrierFunctionRadiusMultiplier*r_a)^2;
                    a_ij = 2*v_ij.'*v_ij + 2*(l1-d/m)*xi_ij.'*v_ij + l0*h;
                    b_ij = 2*1/m*xi_ij.';

                    A = [A; -b_ij];
                    b = [b; mu(i,j)*a_ij];

                    hi = 1
                if norm(xi_ij) < 2*r_a
                    warning('crash')
                end
            end
        end
        F = -2*u_nom(:,i);
        % Add input constraints, to prevent u from exploding
        A = [A; [1, 0; 0, -1]];
        b = [b; u_max; u_max];
        u(:,i) = quadprog(H, F, A, b);
    end

    u_nom_save=[u_nom_save, reshape(u_nom, dimensions*N_a, 1)];
    u_save=[u_save, reshape(u, dimensions*N_a, 1)];

    %% ODE
    % Actual trajectory
    dpdt(1,:) = p(3,:);                           
    dpdt(2,:) = p(4,:);
    dpdt(3,:) = -d/m*p(3,:) + 1/m * u(1,:);
    dpdt(4,:) = -d/m*p(4,:) + 1/m * u(2,:);
    dpdt = reshape(dpdt, [], 1);
end