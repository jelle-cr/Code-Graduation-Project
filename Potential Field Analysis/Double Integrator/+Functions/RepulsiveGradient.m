function [grad_U_rep, rho, rho_m] = RepulsiveGradient(q_i, q_j, n, k_rep, rho_0, r_a, r_o, a_max)
    p_i = q_i(1:2);
    v_i = q_i(3:4);
    p_j = q_j(1:2);
    v_j = q_j(3:4);  

    p_ij = p_i - p_j;
    v_ij = v_i - v_j;
    p_ij_norm = norm(p_ij);             % To avoid unnecessary recalculation
    p_ij_hat = -p_ij/p_ij_norm;
    v_r_ij = v_ij.'*p_ij_hat;
    rho = p_ij_norm - r_a - r_o;  
    rho_m = v_r_ij^2/(2*a_max);
    grad_U_rep = zeros(n,1);
    if rho - rho_m < rho_0
        if v_r_ij > 0
            grad_U_rep_base = -k_rep/(rho-rho_m)^2*(1/(rho-rho_m)-1/rho_0);
            grad_U_rep_p = grad_U_rep_base*(v_r_ij/a_max*(v_ij - v_r_ij*p_ij_hat)/p_ij_norm - p_ij_hat);
            grad_U_rep_v = grad_U_rep_base*(-v_r_ij/a_max*p_ij_hat);
            if rho - rho_m >= 0
                grad_U_rep = [grad_U_rep_p; grad_U_rep_v];
            else   % undefined, this is just filler    
                % warning('Collision guaranteed, flipping repulsive force')
                grad_U_rep = -[grad_U_rep_p; grad_U_rep_v];    % Collision guaranteed (Also flip sign of repulsive force)
            end
            if rho < 0
                warning('########################## Collision ################################')
            end
        end
    end
end