function geom = arm_geometry(group, link_lengths, joint_axes)
    % axis mapping
    joint_axis = struct('x', group.g_circ([0 0 0 1 0 0]),...
                        'y', group.g_circ([0 0 0 0 1 0]),...
                        'z', group.g_circ([0 0 0 0 0 1]));
    %% construct arm
    % assume joints interspersed by links
    geom.joints = cell(size(link_lengths));
    geom.links = cell(size(link_lengths));
    for i = 1:numel(link_lengths)
        geom.links{i} = group.g_mat([link_lengths(i) zeros(1,5)]);
        geom.joints{i} = joint_axis.(joint_axes(i));
    end
end