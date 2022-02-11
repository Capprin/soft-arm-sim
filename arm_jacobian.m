% produces Jacobian based on a single joint between each link
% should take \alpha+\beta as input, and results should be stacked L->R
% \alpha, \beta jacobian terms should be identical
function J_cell = arm_jacobian(group, geometry, joint_angles)
    % assume link, joint correspondence
    % shvars is twice joints; should only need joints Jacobians
    J_cell = cell(size(joint_angles));
    for i = 1:numel(J_cell)
        J_cell{i} = zeros(group.dim, length(joint_angles));
        
    end
end