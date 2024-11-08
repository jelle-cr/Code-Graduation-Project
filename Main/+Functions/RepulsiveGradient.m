function [F_rep, h] = RepulsiveGradient(q, q_o, n, k_rep, rho_0, r_a, r_o, a_max)
    F_rep = zeros(n,1);
    h = 0;
    if n == 2           %Single integrator
        p_i = q(1:2);
        p_j = q_o(1:2);  
    
        p_ij = p_i - p_j;
        p_ij_norm = norm(p_ij);
        rho = p_ij_norm - r_a - r_o;
        h = rho;
        if h < rho_0
            F_rep = -k_rep/h^2*(1/h-1/rho_0)*p_ij/p_ij_norm;
            if rho < 0
                F_rep = -F_rep;   %Flip direction to get unstuck in sim
                warning('########################## Collision ################################')
            end
        end
    elseif n==4         %Double integrator
        p_i = q(1:2);
        v_i = q(3:4);
        p_j = q_o(1:2);
        v_j = q_o(3:4);  
    
        p_ij = p_i - p_j;
        v_ij = v_i - v_j;
        p_ij_norm = norm(p_ij);             % To avoid unnecessary recalculation
        p_ij_hat = -p_ij/p_ij_norm;
        v_r_ij = v_ij.'*p_ij_hat;
        rho = p_ij_norm - r_a - r_o;  
        rho_m = v_r_ij^2/(2*a_max);
        h = rho-rho_m;
        if h < rho_0
            if v_r_ij > 0
                grad_U_rep_base = -k_rep/(h)^2*(1/(h)-1/rho_0);
                grad_U_rep_p = grad_U_rep_base*(v_r_ij/a_max*(v_ij - v_r_ij*p_ij_hat)/p_ij_norm - p_ij_hat);
                grad_U_rep_v = grad_U_rep_base*(-v_r_ij/a_max*p_ij_hat);
                if h >= 0
                    F_rep = [grad_U_rep_p; grad_U_rep_v];
                else   % undefined, this is just filler    
                    warning('Collision guaranteed, flipping repulsive force')
                    F_rep = -[grad_U_rep_p; grad_U_rep_v];    % Collision guaranteed (Also flip sign of repulsive force)
                end
                if rho < 0
                    warning('########################## Collision ################################')
                end
            end
        end
    end
end