% %% Second order (added nominal trajectory)
% function dpdt = odefcn(t,p)      
% 
%     load('./Data/Parameters.mat');
% 
%     p = reshape(p, n, N_a);
%     dpdt = zeros(n, N_a);
% 
%     % Current time
%     t
% 
%     %% Dynamical model
%     f = A*p;
%     g = B;
% 
%     %% Controller
%     u = zeros(m, N_a);
%     u_nom = zeros(m, 1);
%     for i = 1:N_a 
%         V = 1/2*K_att*norm(p(1:2,i)-p_d(1:2,i,floor(t/t_step)+1))^2
%         F_att = K_att*(p(1:2,i)-p_d(1:2,i,floor(t/t_step)+1));
% 
%         L_fV = F_att.'*f(1:2,i);
%         L_f2V = K_att*(p(3,i)^2 + p(4,i)^2);
%         L_gL_fV = F_att.';
% 
%         % a = F_att.'*f(1:2,i);
%         % b = F_att.'*g;
%         % a_tilde = a + norm(b)^2;
% 
%         c_1 = 4;
%         c_0 = 2;
%         a = L_f2V + c_1*L_fV - c_0*V
%         b = L_gL_fV
% 
%         a_tilde = a;
% 
%         if ((a_tilde < 0) || (a_tilde == 0 && norm(b) == 0))
%             u_nom = zeros(m, 1);
%         end
%         if ((a_tilde >= 0) && (norm(b) ~= 0))
%             u_nom = -a_tilde/(norm(b)^2)*b.';
%         end
% 
%         u(:,i) = u_nom;
% 
%         % rho = inf(N_a + N_o,1);   % Initialize all agent and obstacle distances to inf
%         % F_rep = zeros(m,1);
%         % % Compute repulsive force for the agents
%         % for j = 1:N_a
%         %     if j ~= i
%         %         p_ij = p(1:2,i) - p(1:2,j);
%         %         rho(j) = norm(p_ij) - 2*r_a;
%         %         if rho(j) < rho_0 
%         %             if rho(j) >= 0
%         %                 F_rep = F_rep - K_rep/rho(j)^2*(1/rho(j)-1/rho_0)*p_ij/norm(p_ij);
%         %             else        % Agents have collided (Also flip sign of repulsive force)
%         %                 F_rep = F_rep + K_rep/rho(j)^2*(1/rho(j)-1/rho_0)*p_ij/norm(p_ij);
%         %                 % warning(['Collision between drone ' num2str(i) ' and ' num2str(j) ' at time ' num2str(t)])
%         %             end
%         %         end
%         %     end
%         % end
%         % 
%         % % Compute repulsive force for obstacles
%         % if N_o > 0
%         %     for o = 1:N_o
%         %         p_io = p(1:2,i) - p_o(1:2,o);
%         %         rho(N_a+o) = norm(p_io) - r_a - r_o;
%         %         if rho(N_a+o) < rho_0 
%         %             if rho(N_a+o) >= 0 
%         %                 F_rep = F_rep - K_rep/rho(N_a+o)^2*(1/rho(N_a+o)-1/rho_0)*p_io/norm(p_io);
%         %             else       % Agent and obstacle have collided
%         %                 F_rep = F_rep + K_rep/rho(N_a+o)^2*(1/rho(N_a+o)-1/rho_0)*p_io/norm(p_io);
%         %                 % warning(['Collision between drone ' num2str(i) ' and obstacle ' num2str(o) ' at time ' num2str(t)])
%         %             end
%         %         end
%         %     end
%         % end
%         % 
%         % alpha = 1;  % Placeholder, since alpha gets canceled anyways
%         % c = F_rep.'*f(1:2,i) - alpha;
%         % d = F_rep.'*g;
%         % gamma = norm(F_rep)^2 + alpha   - d*u_nom;
%         % c_tilde = c + gamma;
%         % phi = c_tilde + d*u_nom;
%         % 
%         % % Control Barrier Function
%         % if ~APF
%         %     if ((phi < 0) || (phi == 0 && norm(d) == 0))  
%         %         u(:,i) = u_nom;
%         %     elseif ((phi >= 0) && (norm(d) ~= 0))
%         %         u(:,i) = u_nom - phi/norm(d)^2*d.';
%         %     end
%         % end
%         % 
%         % % Artificial Potential Field
%         % if APF
%         %     if rho >= rho_0
%         %         u(:,i) = u_nom;
%         %     else 
%         %         u(:,i) = u_nom - phi/norm(d)^2*d.';
%         %     end
%         % end
% 
%         % Limit control force
%         u(:,i) = min(max(u(:,i), -u_max), u_max);
%     end 
% 
%     %% ODE
%     for i = 1:N_a
%         dpdt(:,i) = A*p(:,i) + B*u(:,i);
%     end
%     dpdt = reshape(dpdt, [], 1);
% end


%% Second order (added nominal trajectory)
function dpdt = odefcn(t,p)      

    load('./Data/Parameters.mat');

    p = reshape(p, n, N_a);
    dpdt = zeros(n, N_a);

    % Current time
    % t

    %% Dynamical model
    f = A*p;
    g = B;

    %% Controller
    u = zeros(m, N_a);
    u_nom = zeros(m, 1);
    for i = 1:N_a 
        F_att = K_att*(p(1:2,i)-p_d(1:2,i,floor(t/t_step)+1));

        a = F_att.'*f(1:2,i);
        b = F_att.'*g;
        a_tilde = a + norm(b)^2;

        if ((a_tilde < 0) || (a_tilde == 0 && norm(b) == 0))
            u_nom = zeros(m, 1);
        end
        if ((a_tilde >= 0) && (norm(b) ~= 0))
            u_nom = -a_tilde/(norm(b)^2)*b.';
        end

        u(:,i) = u_nom;

        rho = inf(N_a + N_o,1);   % Initialize all agent and obstacle distances to inf
        F_rep = zeros(m,1);
        % Compute repulsive force for the agents
        for j = 1:N_a
            if j ~= i
                p_ij = p(1:2,i) - p(1:2,j);
                rho(j) = norm(p_ij) - 2*r_a;
                if rho(j) < rho_0 
                    if rho(j) >= 0
                        F_rep = F_rep - K_rep/rho(j)^2*(1/rho(j)-1/rho_0)*p_ij/norm(p_ij);
                    else        % Agents have collided (Also flip sign of repulsive force)
                        F_rep = F_rep + K_rep/rho(j)^2*(1/rho(j)-1/rho_0)*p_ij/norm(p_ij);
                        % warning(['Collision between drone ' num2str(i) ' and ' num2str(j) ' at time ' num2str(t)])
                    end
                end
            end
        end

        % Compute repulsive force for obstacles
        if N_o > 0
            for o = 1:N_o
                p_io = p(1:2,i) - p_o(1:2,o);
                rho(N_a+o) = norm(p_io) - r_a - r_o;
                if rho(N_a+o) < rho_0 
                    if rho(N_a+o) >= 0 
                        F_rep = F_rep - K_rep/rho(N_a+o)^2*(1/rho(N_a+o)-1/rho_0)*p_io/norm(p_io);
                    else       % Agent and obstacle have collided
                        F_rep = F_rep + K_rep/rho(N_a+o)^2*(1/rho(N_a+o)-1/rho_0)*p_io/norm(p_io);
                        % warning(['Collision between drone ' num2str(i) ' and obstacle ' num2str(o) ' at time ' num2str(t)])
                    end
                end
            end
        end

        alpha = 1;  % Placeholder, since alpha gets canceled anyways
        c = F_rep.'*f(1:2,i) - alpha;
        d = F_rep.'*g;
        gamma = norm(F_rep)^2 + alpha   - d*u_nom;
        c_tilde = c + gamma;
        phi = c_tilde + d*u_nom;

        % Control Barrier Function
        if ~APF
            if ((phi < 0) || (phi == 0 && norm(d) == 0))  
                u(:,i) = u_nom;
            elseif ((phi >= 0) && (norm(d) ~= 0))
                u(:,i) = u_nom - phi/norm(d)^2*d.';
            end
        end

        % Artificial Potential Field
        if APF
            if rho >= rho_0
                u(:,i) = -F_att;
            else 
                u(:,i) = -F_att - F_rep;
            end
            % if rho >= rho_0
            %     u(:,i) = u_nom;
            % else 
            %     u(:,i) = u_nom - phi/norm(d)^2*d.';
            % end
        end

        % Limit control force
        u(:,i) = min(max(u(:,i), -u_max), u_max);
    end 

    %% ODE
    for i = 1:N_a
        dpdt(:,i) = A*p(:,i) + B*u(:,i);
    end
    dpdt = reshape(dpdt, [], 1);
end