% produces Jacobian based on a single joint between each link
% should take \alpha+\beta as input, and results should be stacked L->R
% \alpha, \beta jacobian terms should be identical
function [J_cell, trans] = arm_jacobian(group, geometry, joint_angles)
    % assume link, joint correspondence
    J_cell = cell(size(joint_angles));
    trans = cell(1, numel(joint_angles)+1);
    % get joint transforms
    joint_trans = cell(size(joint_angles));
    for i = 1:numel(joint_trans)
        joint_trans{i} = expm(geometry.joints{i}*joint_angles(i));
    end
    % compute J of each joint
    for i = 1:numel(J_cell)
        J_cell{i} = zeros(group.dim, length(joint_angles));
        % keep track of current location
        g = eye(size(group.g_mat([0 0 0 0 0 0])));
        % columns of J are contributions from ea. prev. joint
        for j = 1:i
            % compute transform between current location and PoI
            h = eye(size(g));
            for k = j:i
                h = h * joint_trans{k} * geometry.links{k};
            end
            % get Jacobian column as a transform
            J_mat = g * geometry.joints{j} * h;
            % convert to vector
            J_vec = group.g_vec(J_mat);
            J_cell{i}(:,j) = J_vec;
            % update g
            trans{j} = g;
            g = g * joint_trans{j} * geometry.links{j};
        end
    end
    trans{end} = g;
end