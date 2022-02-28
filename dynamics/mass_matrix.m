% computes the mass matrix of the arm
function M = mass_matrix(link_masses, inertia_tensors, J_links)
    % pullback of each link
    M_local = cell(size(J_links));
    for i = 1:numel(J_links)
        % compute local mass matrix
        m = link_masses(i);
        mu = [diag([m m m]) zeros(3); zeros(3) inertia_tensors{i}];
        % pullback
        M_local{i} = J_links{i}' * mu * J_links{i};
    end
    M = cellfun(@sum, M_local);
end