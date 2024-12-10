function [gradU_att, gradU_rep, h] = potential_gradients(m, p, p_d, p_o, k_att, k_rep, r_a, r_o, rho_0)
    gradU_att = zeros(m,1);
    gradU_rep = zeros(m,1);
    
    N_o = width(p_o);
    
    % Attractive gradient
    gradU_att = k_att*(p-p_d);

    % Repulsive gradient
    for j = 1:N_o
        p_ij = p-p_o(:,j);
        h(j) = norm(p_ij) - r_a - r_o;
        if h(j) < rho_0
            gradU_rep = gradU_rep - k_rep/h(j)^2*(1/h(j)-1/rho_0)*p_ij/norm(p_ij);
        end
        % gradU_rep = gradU_rep - k_rep/h(j)^2*p_ij/norm(p_ij); % U_rep=1/h
    end
end