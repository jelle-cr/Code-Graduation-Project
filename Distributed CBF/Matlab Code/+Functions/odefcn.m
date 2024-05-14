%% Second order (added nominal trajectory)
function dXdt = odefcn(t,X)      
    global u_save u_nom_save

    load('./Data/Parameters.mat');

    X = reshape(X, states, N_a);
    dXdt = zeros(states, N_a);
    % Note that [X(1) X(2) X(3) X(4)]==[x y x' y']
    % time
    t

    %% Dynamics
    f = [X(3); X(4); -d/m*X(3); -d/m*X(4)];
    g = [0 0; 0 0; 1/m 0; 0 1/m];

    %% CLF nominal controller
    % Nominal trajectories
    X_nom = Functions.calculate_nominal_trajectories(t);
    u_nom = zeros(dimensions,N_a);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Remove
    % X_nom = [0.4 -0.4; 0 0; 0 0; 0 0];

    H = 2*eye(dimensions);
    F = zeros(1,dimensions);
    for i = 1:N_a
        A = [];
        b = [];
        V = 1/2*(X(1,i)-X_nom(1,i))^2 + 1/2*(X(2,i)-X_nom(2,i))^2 + 1/2*(X(3,i)+l2*(X(1,i)-X_nom(1,i)))^2 + 1/2*(X(4,i)+l3*(X(2,i)-X_nom(2,i)))^2;
        gradV = [X(1,i) - X_nom(1,i) + l2*(X(3,i)-X_nom(3,i)+l2*(X(1,i)-X_nom(1,i)));
                 X(2,i) - X_nom(2,i) + l3*(X(4,i)-X_nom(4,i)+l3*(X(2,i)-X_nom(2,i)));
                 X(3,i) - X_nom(3,i) + l2*(X(1,i)-X_nom(1,i));
                 X(4,i) - X_nom(4,i) + l3*(X(2,i)-X_nom(2,i))];
        L_fV= dot(gradV, f);
        L_gV= [dot(gradV, g(:,1)), dot(gradV, g(:,2))];
        A = L_gV;
        b = -L_fV-lambda*V;
        % Add input constraints, to prevent u_nom from exploding
        A = [A; [1, 0; 0, -1]];
        b = [b; u_max; u_max];

        u_nom(:,i) = quadprog(H, F, A, b);
    end
    % gain = 0.1;
    % u_nom(:,:) = -[gain*(X(1,1)-0.4) gain*(X(1,2)+0.4); 0 0];

    %% CBF Safety filter
    u = zeros(dimensions, N_a);
    H = 2*eye(dimensions);
    for i = 1:N_a
        A = [];
        b = [];
        for j = 1:N_a
            if i ~= j
                p_ij = X(1 : dimensions, i) - X(1 : dimensions, j);
                if (norm(p_ij)-2*r_a) < barrierFunctionMaxDistance
                    v_ij = X(dimensions+1 : 2*dimensions, i) - X(dimensions+1 : 2*dimensions, j);
                    h = p_ij.'*p_ij - (2*barrierFunctionRadiusMultiplier*r_a)^2;
                    a_ij = 2*v_ij.'*v_ij + 2*(l1-d/m)*p_ij.'*v_ij + l0*h;
                    b_ij = 2*1/m*p_ij.';

                    A = [A; -b_ij];
                    b = [b; mu(i,j)*a_ij];

                    % i = i
                    % j = j
                    % p_ij = p_ij
                end
                if norm(p_ij) < 2*r_a
                    warning(['Collision between drone ' num2str(i) ' and ' num2str(j) ' at time ' num2str(t)]);
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
    dXdt(1,:) = X(3,:);                           
    dXdt(2,:) = X(4,:);
    dXdt(3,:) = -d/m*X(3,:) + 1/m * u(1,:);
    dXdt(4,:) = -d/m*X(4,:) + 1/m * u(2,:);
    dXdt = reshape(dXdt, [], 1);
end