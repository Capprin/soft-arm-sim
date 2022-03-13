% computes jacobian in local frame
% almost identical to other jacobians (no g transform)
function [J_cell] = body_jacobian(group, geometry, link_props, joint_angles)
    % setup
    J_cell = cell(size(link_props));
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
    % compute body J at each link
    for i = 1:numel(link_props)
        J_cell{i} = sym(zeros(group.dim, length(joint_angles)));
        % columns of J are contributions from ea. prev. joint
        for j = 1:i
            % compute transform between current location and PoI
            h = group.g_mat(zeros(1,group.dim)); %identity
            for k = j:i
                % use partial link if at end
                if k == i
                    h = h * joint_trans{k} * link_trans{k};
                else
                    h = h * joint_trans{k} * geometry.links{k};
                end
            end
            % get Jacobian column as a transform
            J_mat = h \ (geometry.joints{j} * h);
            % convert to vector
            J_vec = group.g_dot(J_mat);
            J_cell{i}(:,j) = J_vec;
        end
    end
end