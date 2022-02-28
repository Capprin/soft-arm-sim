% computes potential energy term
function PE = potential_energy(K_mat, gamma, link_masses, J_links, q)
    % get contribution of springs
    kt = K_mat * q;
    % get contribution of heights
    inertia = zeros(size(J_links{1}));
    for i = 1:numel(J_links)
        inertia = inertia + J_links{i}' * link_masses{i};
    end
    ht = gamma * inertia * [0 1 0]';
    % contribution from potential is sum of torques
    PE = kt + ht;
end