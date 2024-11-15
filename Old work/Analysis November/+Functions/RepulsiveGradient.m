function gradU_rep = RepulsiveGradient(p, p_o, n, k_rep, rho_0, r_a, r_o, t)
    gradU_rep = zeros(2,1);
    p_ij = p - p_o;
    h = norm(p_ij) - r_a - r_o;

    if  h < rho_0
        gradU_rep = -k_rep/(h^2)*(1/h-1/rho_0)*p_ij/norm(p_ij);
        if h < 0
            warning(['Collision at time ' num2str(t) ' s'])
            gradU_rep = -gradU_rep;
        end
    end
end