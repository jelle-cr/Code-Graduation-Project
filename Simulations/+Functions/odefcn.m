function dxdt = odefcn(t, x)      
    load('./Data/parameters.mat');
    persistent u_save

    x = reshape(x, n, N_a);         % Full state matrix
    % dxdt = zeros(n, 1);

    %% Dynamical model (for nonlinear model, override A)
    % f = A*x;
    % g = B;

    %% Controller
    u = zeros(m, N_a);              % Initialize controller
    
    for i = 1:N_a
        e_p = x(:,i)-x_d;                    % State error
    
        u(:,i) = -e_p;
    end

    %% ODE
    for i = 1:N_a
        dxdt(:,i) = A*x(:,i) + B*u(:,i);
    end
    dxdt = reshape(dxdt, [], 1);

    %% Save control input (assumes ode4)
    u_save(:,:,end+1) = u;

    u = [];
    ind = 1;
    if(t == t_end)
        for i = 1:length(u_save)
            if mod(i,4)==2  % Take every 4th sample, starting from 2
                u(:,:,ind) = u_save(:,:,i);
                ind = ind + 1;
            end
        end
        save('./Data/SimulationData.mat', 'u');
    end
end