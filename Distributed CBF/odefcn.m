%% Second order (added nominal trajectory)
function dpdt = odefcn(t,p)      
    global u_save u_nom

    load('parameters.mat');
    
    p = reshape(p, states, N_a);
    dpdt = zeros(states, N_a);
    % Note that [p(1) p(2) p(3) p(4)]==[x y x' y']
    % time
    t

    % Full responsibility mu = 1, or half responsibility mu = 1/2
    mu = 1/2;

    % Nominal controls
    u0 = sign_rand.*[A_rand; A_rand].*[cos(f_rand*t); sin(f_rand*t)];

    %% Dynamic model
    u = zeros(dimensions, N_a);
    for i = 1:N_a%1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%1:N_a
        A = [];
        b = [];
        for j = 1:N_a
            if i ~= j
                xi_ij = p(1 : dimensions, i) - p(1 : dimensions, j);
                v_ij = p(dimensions+1 : 2*dimensions, i) - p(dimensions+1 : 2*dimensions, j);
                h = xi_ij.'*xi_ij - (2*agent_radius)^2;
                a_ij = 2*v_ij.'*v_ij + 2*(l1-d/m)*xi_ij.'*v_ij + l0*h;
                b_ij = 2*1/m*xi_ij.';

                A = [A; -b_ij];
                b = [b; mu*a_ij];
            end
        end
        H = 2*eye(dimensions);
        f = -2*u0(:,i);
        u(:,i) = quadprog(H, f, A, b);
    end

    u_nom=[u_nom, reshape(u0, dimensions*N_a, 1)];
    u_save=[u_save, reshape(u, dimensions*N_a, 1)];

    %% Dynamics of the system
    % Actual trajectory
    dpdt(1,:) = p(3,:);                           
    dpdt(2,:) = p(4,:);
    dpdt(3,:) = -d/m*p(3,:) + 1/m * u(1,:);
    dpdt(4,:) = -d/m*p(4,:) + 1/m * u(2,:);
    dpdt = reshape(dpdt, [], 1);
end