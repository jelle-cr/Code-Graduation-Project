function p_0 = generate_initial_positions(N_a, r_a, range, p_o, r_o)
    % Initialize the positions matrix
    p_0 = zeros(2, N_a);
    
    % Number of obstacles
    N_o = size(p_o, 2);
    
    % Maximum number of attempts to find a non-overlapping position with agents
    max_attempts = 20;
    
    % Function to check if two circles overlap
    function overlap = check_overlap(pos1, r1, pos2, r2)
        distance = sqrt((pos1(1) - pos2(1))^2 + (pos1(2) - pos2(2))^2);
        overlap = distance < (r1 + r2);
    end
    
    agent_index = 1;
    
    while agent_index <= N_a
        attempt = 0;
        success = false;
        
        while attempt < max_attempts
            % Generate a random position within the box
            potential_x = -range + 2 * range * rand;
            potential_y = -range + 2 * range * rand;
            potential_pos = [potential_x; potential_y];
            
            % Check overlap with obstacles (always avoid)
            overlap_with_obstacles = false;
            for j = 1:N_o
                if check_overlap(potential_pos, r_a, p_o(:, j), r_o)
                    overlap_with_obstacles = true;
                    break;
                end
            end
            
            % If there is an overlap with obstacles, try a new position
            if overlap_with_obstacles
                attempt = attempt + 1;
                continue;
            end
            
            % Check overlap with other agents
            overlap_with_agents = false;
            for i = 1:(agent_index - 1)
                if check_overlap(potential_pos, r_a, p_0(:, i), r_a)
                    overlap_with_agents = true;
                    break;
                end
            end
            
            % If no overlap with agents or obstacles, place the agent
            if ~overlap_with_agents
                p_0(:, agent_index) = potential_pos;
                success = true;
                break;
            end
            
            attempt = attempt + 1;
        end
        
        % If max_attempts reached without success (overlap with agents allowed)
        if ~success
            while true
                % Generate a new random position that does not overlap with obstacles
                potential_x = -range + 2 * range * rand;
                potential_y = -range + 2 * range * rand;
                potential_pos = [potential_x; potential_y];
                
                % Check overlap with obstacles
                overlap_with_obstacles = false;
                for j = 1:N_o
                    if check_overlap(potential_pos, r_a, p_o(:, j), r_o)
                        overlap_with_obstacles = true;
                        break;
                    end
                end
                
                % If no overlap with obstacles, place the agent, even if it overlaps with other agents
                if ~overlap_with_obstacles
                    p_0(:, agent_index) = potential_pos;
                    break;
                end
            end
        end
        
        agent_index = agent_index + 1;
    end
end
