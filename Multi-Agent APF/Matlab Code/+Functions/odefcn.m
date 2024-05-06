%% Second order (added nominal trajectory)
function dpdt = odefcn(t,p)      
    global u_att_save u_rep_save

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
    u = zeros(dimensions, N_a);
    
    u_att = zeros(dimensions,N_a);
    u_rep = zeros(dimensions,N_a);

    for i = 1:N_a    
        U_att = 1/2*K_att*norm(p(1:2,i)-p_nom(1:2,i))^2;
        F_att = K_att*(p(1:2,i)-p_nom(1:2,i));
        U_rep = 0;
        F_rep = 0;
        for j = 1:N_a
            if i ~= j
                rho = norm(p(1:2,i)-p(1:2,j)) - 2*r_a;
                if rho < rho_0
                    U_rep = U_rep + 1/2*K_rep*(1/rho-1/rho_0)^2;
                    F_rep = F_rep - K_rep/rho^2*(1/rho-1/rho_0)*(p(1:2,i)-p(1:2,j))/norm(p(1:2,i)-p(1:2,j));
                end
            end
        end
        u_att(:,i) = -F_att;
        u_rep(:,i) = -F_rep;
        u(:,i) = u_att(:,i) + u_rep(:,i);
    end    

    u_att_save=[u_att_save, reshape(u_att, dimensions*N_a, 1)];
    u_rep_save=[u_rep_save, reshape(u_rep, dimensions*N_a, 1)];

    %% ODE
    % Actual trajectory
    dpdt(1,:) = p(3,:);                           
    dpdt(2,:) = p(4,:);
    dpdt(3,:) = -d/m*p(3,:) + 1/m * u(1,:);
    dpdt(4,:) = -d/m*p(4,:) + 1/m * u(2,:);
    dpdt = reshape(dpdt, [], 1);
end