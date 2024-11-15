function p_0 = generate_initial_positions(model, N_a, r_a, formationDistance, p_o, r_o)
    % Calculate the initial number of agents along one side of the grid
    side_length = ceil(sqrt(N_a));
    
    % Initialize the positions matrix
    p_0 = zeros(2, N_a);
    
    % Variable to keep track of agent count
    agent_index = 1;
    
    while agent_index <= N_a
        % Calculate the starting point to center the grid around the origin
        start_x = -((side_length - 1) / 2) * formationDistance;
        start_y = -((side_length - 1) / 2) * formationDistance;
        
        % Reset the positions matrix
        p_0 = zeros(2, N_a);
        agent_index = 1;
        
        % Generate the grid positions
        for i = 0:(side_length-1)
            for j = 0:(side_length-1)
                if agent_index > N_a
                    break;
                end
                % Calculate potential position
                potential_x = start_x + i * formationDistance;
                potential_y = start_y + j * formationDistance;
                
                % Check for overlap with any obstacle
                overlap = false;
                for k = 1:size(p_o, 2)
                    distance = sqrt((potential_x - p_o(1, k))^2 + (potential_y - p_o(2, k))^2);
                    if distance < (r_a + r_o)
                        overlap = true;
                        break;
                    end
                end
                
                % If no overlap, assign position to the agent
                if ~overlap
                    p_0(1, agent_index) = potential_x;
                    p_0(2, agent_index) = potential_y;
                    agent_index = agent_index + 1;
                end
            end
            if agent_index > N_a
                break;
            end
        end
        
        % Increase grid size if not all agents have been placed
        if agent_index <= N_a
            side_length = side_length + 1;
        end
    end
    if strcmp(model, 'doubleIntegrator')
        p_0 = [p_0; zeros(2,N_a)];
    end
end