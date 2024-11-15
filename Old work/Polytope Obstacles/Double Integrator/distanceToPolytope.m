function [A_closest, b_closest] = distanceToPolytope(p_i, r_a)

    % Define the vertices of the polytope
    poly = [-1, 1, -1;
            -2, 1, 1];
    
    % Number of vertices
    n_v = size(poly, 2);
    
    % Initialize matrices
    A = zeros(n_v, 2);
    b = zeros(n_v, 1);
    
    for i = 1:n_v
        % Get the current, next, and previous vertices
        p1 = poly(:, i);
        p2 = poly(:, mod(i, n_v) + 1);
        p3 = poly(:, mod(i + 1, n_v) + 1);
    
        % Compute the edge direction and normal
        A(i, :) = [p2(2) - p1(2), p1(1) - p2(1)];
        b(i, 1) = p2(1) * p1(2) - p1(1) * p2(2);
        
        % Check which side of the inequality the third vertex lies on
        if A(i, :) * p3 + b(i) > 0
            A(i, :) = -A(i, :);
            b(i, 1) = -b(i, 1);
        end
    end
    [d,i_c] = max(A*p_i + b);

    % Define the points p1 and p2 for the closest edge
    p1 = poly(:, i_c);
    p2 = poly(:, mod(i_c, n_v) + 1);
    
    % Compute the vector from p1 to p2
    edge_vec = p2 - p1;
    
    % Project point p_i onto the line through p1 and p2
    t = ( (p_i - p1)' * edge_vec ) / (edge_vec' * edge_vec);
    
    % Check if the projection lies within the edge (0 <= t <= 1)
    if t < 0
        p_closest = p1;  % Closest point is p1
    elseif t > 1
        p_closest = p2;  % Closest point is p2
    else
        p_closest = p1 + t * edge_vec;  % Closest point is on the edge
    end

    dist = d/norm(A(i_c,:)) - r_a
    A_closest = A(i_c,:);
    b_closest = b(i_c,1);

end
  