%% Second order (added nominal trajectory)
function dpdt = odefcn(t,p_combined)      
    global u_save u_nom

    load('parameters.mat');
    
    p_combined = reshape(p_combined, height(p_combined)/N_a, N_a)
    p = p_combined(1:states,:)
    p0 = p_combined(states+1:2*states,:)
    dpdt = zeros(2*height(p),N_a);
    % Note that [p(1) p(2) p(3) p(4)]==[x y x' y']
    % time
    t

    % Full responsibility mu = 1, or half responsibility mu = 1/2
    mu = 1;

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
                % v_ij = p(dimensions+1 : 2*dimensions, i) - p(dimensions+1 : 2*dimensions, j);
                h = xi_ij.'*xi_ij - (2*agent_radius)^2;
                grad_h = [2*xi_ij; 0; 0];
                L_fh = 2*[p(3,i), p(4,i)]*xi_ij;
                L_f2h = 2*p(3,i)^2 + 2*p(4,i)^2 - 2*d/m*xi_ij.'*[p(3,i); p(4,i)];
                L_gL_fh = 2/m*xi_ij.';

                A = [A; -L_gL_fh];
                b = [b; L_f2h + l1*L_fh + l0*h];
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
    % Nominal trajectory
    dpdt(5,:) = p0(3,:);                           
    dpdt(6,:) = p0(4,:);
    dpdt(7,:) = -d/m*p0(3,:) + 1/m * u0(1,:);
    dpdt(8,:) = -d/m*p0(4,:) + 1/m * u0(2,:);
    dpdt
    dpdt = reshape(dpdt, [], 1);
    dpdt
end