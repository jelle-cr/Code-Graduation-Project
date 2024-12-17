function [gradU_att, gradU_rep] = calculateSIgradients(p,p_d,p_o)
    load('./Data/parameters.mat');
    gradU_att = zeros(m,N_a);
    gradU_rep = zeros(m,N_a);

    for i = 1:N_a 
        % Attractive gradient
        gradU_att(:,i) = k_att*(p(:,i)-p_d);
           
        % Repulsive gradient
        for j = 1:N_o
            p_ij = p(:,i)-p_o(:,j);
            h = norm(p_ij) - r_a - r_o;
            if h < rho_0
                gradU_rep(:,i) = gradU_rep(:,i) - k_rep/h^2*(1/h-1/rho_0)*p_ij/norm(p_ij);
            end
        end
    end
end