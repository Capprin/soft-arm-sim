% produces Jacobian of specified points along each link
% similar to arm_jacobian.m, with additional partial-link logic
function [J_cell, trans] = link_jacobian(group, geometry, joint_angles, link_props)
    % setup
    J_cell = cell(size(link_props));
    trans = cell(size(link_props));
    % get joint transforms
    joint_trans = cell(size(joint_angles));
    for i = 1:numel(joint_trans)
        joint_trans{i} = expm(geometry.joints{i}*joint_angles(i));
    end
    % get partial links
    link_trans = cell(size(link_props));
    for i = 1:numel(link_props)
        mask_row = [1 1 1 link_props(i)];
        mask = [mask_row; mask_row; mask_row; 1 1 1 1];
        link_trans{i} = geometry.links{i} .* mask;
    end
    % compute J at each partial link
    for i = 1:numel(link_props)
        J_cell{i} = zeros(group.dim, length(joint_angles));
        % keep track of current location
        g = eye(size(group.g_mat([0 0 0 0 0 0])));
        % columns of J are contributions from ea. prev. joint
        for j = 1:i
            % compute transform between current location and PoI
            h = eye(size(g));
            for k = j:i
                % use partial link if at end
                if k == i
                    h = h * joint_trans{k} * link_trans{k};
                else
                    h = h * joint_trans{k} * geometry.links{k};
                end
            end
            % get Jacobian column as a transform
            J_mat = g * geometry.joints{j} * h;
            % convert to vector
            J_vec = group.g_vec(J_mat);
            J_cell{i}(:,j) = J_vec;
            % update g
            trans{j} = g * joint_trans{j} * link_trans{j};
            g = g * joint_trans{j} * geometry.links{j};
        end
    end
end