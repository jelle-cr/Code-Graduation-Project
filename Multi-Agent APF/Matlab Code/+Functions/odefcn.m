%% Second order (added nominal trajectory)
function dXdt = odefcn(t,X)      
    global u_att_save u_rep_save

    load('./Data/Parameters.mat');

    X = reshape(X, states, N_a);
    dXdt = zeros(states, N_a);
    % Note that [X(1) X(2) X(3) X(4)]==[x y x' y']
    % time
    t

    %% CLF nominal controller
    % Nominal trajectories
    X_nom = Functions.calculate_nominal_trajectories(t);
    % X_nom = [0.4 -0.4; 0 0; 0 0; 0 0];
    u = zeros(dimensions, N_a);
    
    u_att = zeros(dimensions,N_a);
    u_rep = zeros(dimensions,N_a);

    for i = 1:N_a    
        % U_att = 1/2*K_att_p*norm(X(1:2,i)-X_nom(1:2,i))^2 + 1/2*K_att_v*norm(X(3:4,i)-X_nom(3:4,i))^2;;
        F_att = K_att_p*(X(1:dimensions,i)-X_nom(1:dimensions,i)) + K_att_v*(X(dimensions+1:2*dimensions,i)-X_nom(dimensions+1:2*dimensions,i));

        U_rep = zeros(dimensions, 1);
        F_rep = zeros(dimensions, 1);
        for j = 1:N_a
            if i ~= j
                p_ij = X(1:dimensions,i)-X(1:dimensions,j);
                v_ij = X(dimensions+1:2*dimensions,i) - X(dimensions+1:2*dimensions,j);
                n_ij = -p_ij/norm(p_ij);
                v_r_ij = v_ij.'*n_ij;
                rho = norm(p_ij) - 2*r_a;
                rho_m = v_r_ij^2/(2*a_max);
                vn_perp = v_ij - v_r_ij*n_ij;

                if rho < 0              % Agents have collided
                    warning(['Collision between drone ' num2str(i) ' and ' num2str(j) ' at time ' num2str(t)]);
                    rho = norm(p_ij);  % Attempt to steer away from the center of the other agent
                end

                % if (rho-rho_m >= rho_0 || v_r_ij <= 0)
                %     F_rep = F_rep - K_rep/rho^2*(1/rho-1/rho_0)*(X(1:2,i)-X(1:2,j))/norm(X(1:2,i)-X(1:2,j));
                % end
                if (rho-rho_m < rho_0 && v_r_ij > 0)
                    F_rep = F_rep-K_rep/(rho-rho_m)^2*(1/(rho-rho_m)-1/rho_0)*((-v_r_ij/a_max-2*rho_m/norm(p_ij)-1)*n_ij+v_r_ij/(a_max*norm(p_ij))*v_ij);
                    % size1 = size(F_rep1)
                    % size2 = size(F_rep2)
                    % F_rep1 = -K_rep/(rho-rho_m)^2*(1+v_r_ij/a_max)*n_ij;
                    % F_rep2 = K_rep*v_r_ij/(rho*a_max*(rho-rho_m)^2)*vn_perp;
                    % F_rep = F_rep - F_rep1 - F_rep2;
                end
                if (rho < rho_m && v_r_ij > 0)
                    F_rep = F_rep;
                    warning('Crash Imminent%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
                end
                % if rho < rho_0
                %     % U_rep = U_rep + 1/2*K_rep*(1/rho-1/rho_0)^2;
                %     F_rep = F_rep - K_rep/rho^2*(1/rho-1/rho_0)*(-n_ij);
                % end
            end
        end
        u_att(:,i) = -min(max(F_att, -u_max), u_max);   % Limit attractive and repulsive forces
        u_rep(:,i) = -min(max(F_rep, -u_max), u_max);   % Note that u can still be > u_max due to the sum
        u(:,i) = u_att(:,i) + u_rep(:,i);
    end    

    u_att_save=[u_att_save, reshape(u_att, dimensions*N_a, 1)];
    u_rep_save=[u_rep_save, reshape(u_rep, dimensions*N_a, 1)];

    %% ODE
    % Actual trajectory
    dXdt(1,:) = X(3,:);                           
    dXdt(2,:) = X(4,:);
    dXdt(3,:) = -d/m*X(3,:) + 1/m * u(1,:);
    dXdt(4,:) = -d/m*X(4,:) + 1/m * u(2,:);
    dXdt = reshape(dXdt, [], 1);
end