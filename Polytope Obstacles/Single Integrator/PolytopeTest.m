clear all
% Define the vertices of the polytope
poly = [1, 3, 2;
        1, 2, 4];

p_i = [3;-1];

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

    dist = (A(i,:)*p_i+b(i,1))/norm(A(i,:))
end
