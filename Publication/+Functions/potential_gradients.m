function [F_att, F_rep, h] = potential_gradients(m, p, p_d, p_o, k_att, k_rep, r_a, r_o, h_0)
    F_att = zeros(m, 1);
    F_rep = zeros(m, 1);
    
    N_o = width(p_o);
    
    % Attractive gradient
    F_att = k_att*(p - p_d);

    % Repulsive gradient
    h = zeros(N_o, 1);
    for j = 1:N_o
        p_ij = p - p_o(:,j);
        h(j) = norm(p_ij) - r_a - r_o;
        if h(j) < h_0
            F_rep = F_rep - k_rep/h(j)^2*(1/h(j)-1/h_0)*p_ij/norm(p_ij);
        end
    end
end