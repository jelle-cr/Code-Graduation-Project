function p = generate_circular_initial_positions(N_a, r_a, agent_spacing)
    % Calculate formation circle radius   
    formation_radius = r_a + agent_spacing*N_a*r_a;
    
    % Generate angles for agents around the circle
    theta = linspace(0, 2*pi*(N_a - 1)/N_a, N_a);
    
    % Convert angles to Cartesian coordinates
    x = formation_radius * cos(theta);
    y = formation_radius * sin(theta);
    
    % Form the position matrix (x coordinates in the first row, y in the second)
    p = [x; y];
end