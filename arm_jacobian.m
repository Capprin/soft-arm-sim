function J_cell = arm_jacobian(group, geometry, shvars)
    % assume link, joint correspondence
    J_cell = cell(size(joint_angles));
    for i = 1:numel(J_cell)
        J_cell{i} = zeros(group.dim, length(shvars))
    end
end