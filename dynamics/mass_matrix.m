% computes the mass matrix of the arm
function M = mass_matrix(link_masses, inertia_tensors, J_links)
    % pullback of each link
    M = sym(zeros(2*size(inertia_tensors{1})));
    for i = 1:numel(J_links)
        % compute local mass matrix
        m = link_masses(i);
        mu = [diag([m m m]) zeros(3); zeros(3) inertia_tensors{i}];
        % pullback
        M_local = J_links{i}' * mu * J_links{i};
        M = M + M_local;
    end
end