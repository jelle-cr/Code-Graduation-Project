function X_d = calculate_desired_trajectories(t)
    load('./Data/TrajectoryParameters.mat');

    % Reference position
    p_ref = sign_rand.*[A_rand; A_rand].*[cos(f_rand*t + phi_rand); sin(f_rand*t + phi_rand)] + origin_rand;

    % Reference velocity (derivative of position)
    if use_V_ref
        v_ref = sign_rand.*[A_rand; A_rand].*[-f_rand.*sin(f_rand*t + phi_rand); f_rand.*cos(f_rand*t + phi_rand)];
    else
        v_ref = [];
    end

    % Reference states
    X_d = [p_ref; v_ref];
end