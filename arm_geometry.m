function geom = arm_geometry(group, link_axes, link_lengths, joint_axes)
    % axis mappings
    link_axis = struct('x', [1 0 0 0 0 0],...
                       'y', [0 1 0 0 0 0],...
                       'z', [0 0 1 0 0 0]);
    joint_axis = struct('x', [0 0 0 1 0 0],...
                        'y', [0 0 0 0 1 0],...
                        'z', [0 0 0 0 0 1]);
    %% construct arm
    % assume joints interspersed by links
    geom.joints = cell(size(link_lengths));
    geom.links = cell(size(link_lengths));
    for i = 1:numel(link_lengths)
        geom.links{i} = group.g_mat(link_lengths(i) * link_axis.(link_axes{i}));
        geom.joints{i} = group.g_circ(joint_axis.(joint_axes{i}));
    end
end