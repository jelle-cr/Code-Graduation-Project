function p_d = generate_desired_positions(n, N_a, t, numWaypoints, range)

    p_d = zeros(n, N_a, length(t));
    
    % Generate trajectories for each robot
    for i = 1:N_a
        % Generate random waypoints within the specified range
        waypoints = rand(n, numWaypoints) .* (range(:, 2) - range(:, 1)) + range(:, 1);
        tWaypoints = linspace(0, 1, numWaypoints);
        tSteps = linspace(0, 1, length(t));
        
        % Spline interpolation to create smooth trajectories
        xTrajectory = spline(tWaypoints, waypoints(1, :), tSteps);
        yTrajectory = spline(tWaypoints, waypoints(2, :), tSteps);
        
        % Store the trajectories in the 3D matrix
        p_d(1, i, :) = xTrajectory;
        p_d(2, i, :) = yTrajectory;
    end
end