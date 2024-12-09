function [x_0, x_d, x_o] = environment_setup(environment, dynamics, N_a)
    x_0 = 0;
    x_d = 0;
    x_o = 0;
    % if strcmp(dynamics, 'Single Integrator')
        if strcmp(environment, 'tripleObstacle')
            % x_0 = [-2.5;        % Initial position
            %        -1.5];
            x_0 = [-4;        % Initial position
                   -2];
            if N_a == 2
                x_0 = [-2.5, 0;     % Initial positions
                       -1.5, -1];
            end
            % x_d = [2;           % Desired position
            %        1];
            % x_o = [-1, 0, 1.5;           % Obstacle positions
            %        -1, 0.5, -0.25];
            x_d = [3.5;           % Desired position
                   2];
            x_o = [-2, -0.5, 2;           % Obstacle positions
                   -1, 1.25, 0.75];
        end
        if strcmp(environment, 'corridor')
            x_0 = [-2.5;        % Initial position
                   -1.5];
            x_d = [2;           % Desired position
                   0.25];
            x_o = [0, 0;           % Obstacle positions
                   -0.85, 0.85];
        end
        if strcmp(environment, 'goalNearObstacle')
            x_0 = [-2.5;        % Initial position
                   -1.5];
            x_d = [0;           % Desired position
                   0.80];
            x_o = [0;           % Obstacle positions
                   0];
        end
        if strcmp(environment, 'singletary')
            x_0 = [0;        % Initial position
                   0];
            x_d = [3;           % Desired position
                   5];
            x_o = [1, 2.5;           % Obstacle positions
                   2, 3];
        end
    % end
end

