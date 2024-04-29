function mu = calculate_agent_mu(N_a, agent_responsibility_weights)
    % Parameter mu determines how much responsibility each agent takes for collision avoidance
    % For DR protocol, mu(i,j) + mu(j,i) == 1

    mu = zeros(N_a);
    for i = 1:N_a
        for j = i+1:N_a
            weight_ratio = agent_responsibility_weights(i) / (agent_responsibility_weights(i) + agent_responsibility_weights(j));
            mu(i,j) = weight_ratio;
            mu(j,i) = 1-weight_ratio;   % DR protocol
        end
    end
end