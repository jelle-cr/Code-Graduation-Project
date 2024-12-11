function [u_att, u_rep] = APF_safety_filter(m, F_att, F_rep, sigma, gamma, alpha)
    u_att = zeros(m,1);
    u_rep = zeros(m,1);

    K_att = sigma/norm(F_att)^2;
    u_att = K_att*F_att;

    K_rep = (gamma - alpha - K_att*F_rep.'*F_att)/norm(F_rep)^2;
    if K_rep > 0
        u_rep = K_rep*F_rep;           % Original
        % u_rep = min(1,K_rep)*F_rep;    % Saturated
    end
end

