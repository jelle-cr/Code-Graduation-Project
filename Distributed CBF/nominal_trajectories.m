function p_nom = nominal_trajectories(t)
    load('TrajectoryParameters.mat');
    p_nom = sign_rand.*[A_rand; A_rand].*[cos(f_rand*t + phi_rand); sin(f_rand*t + phi_rand)] + origin_rand;
end