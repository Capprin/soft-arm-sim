% kinematic_test.m
% tests group, geometry, and Jacobian

% initialization
link_axes = {'z', 'z', 'z'};
link_lengths = [0.1 0.5 0.5];
link_coms = [0.5 0.5 0.5];
joint_axes = {'z', 'y', 'y'};
joint_angles = [-pi pi/4 pi/2];

% put together geometry
group = SE3;
geom = arm_geometry(group, link_axes, link_lengths, joint_axes);

% get Jacobian, global joint transforms
[J, joint_trans] = arm_jacobian(group, geom, joint_angles);
[J_com, link_trans] = link_jacobian(group, geom, link_coms, joint_angles);
J_bod = body_jacobian(group, geom, link_coms, joint_angles);

% get global joint locations
joint_vecs = zeros(3,numel(joint_trans));
for i = 1:numel(joint_trans)
    se3_vec = group.g_vec(joint_trans{i});
    joint_vecs(:,i) = se3_vec(1:3);
end
% get global link locations
link_vecs = zeros(3,numel(link_trans));
for i = 1:numel(link_trans)
    se3_vec = group.g_vec(link_trans{i});
    link_vecs(:,i) = se3_vec(1:3);
end

% plot
% setup
figure(7);clf;
hold on;grid on; box on; axis equal;
l = sum(link_lengths);
xlim([-l l]); ylim([-l l]); zlim([-l l])
xlabel('x'); ylabel('y'); zlabel('z');
view(3);
% arm
plot3(joint_vecs(1,:), joint_vecs(2,:), joint_vecs(3,:), 'Color', 'k',...
      'LineStyle', '-', 'Marker', '.', 'LineWidth', 5, 'MarkerSize', 30);
% Jacobians
% for i = 1:numel(J)
%     % one arrow for ea. joint contribution
%     quiver3(joint_vecs(1,i+1), joint_vecs(2,i+1), joint_vecs(3,i+1),...
%             J{i}(1,1), J{i}(2,1), J{i}(3,1),'r', 'LineWidth', 2);
%     quiver3(joint_vecs(1,i+1), joint_vecs(2,i+1), joint_vecs(3,i+1),...
%             J{i}(1,2), J{i}(2,2), J{i}(3,2),'y', 'LineWidth', 2);
%     quiver3(joint_vecs(1,i+1), joint_vecs(2,i+1), joint_vecs(3,i+1),...
%             J{i}(1,3), J{i}(2,3), J{i}(3,3),'b', 'LineWidth', 2);
% end
for i = 1:numel(J_com)
    % one arrow for ea. joint contribution
    quiver3(link_vecs(1,i), link_vecs(2,i), link_vecs(3,i),...
            J_com{i}(1,1), J_com{i}(2,1), J_com{i}(3,1),'r', 'LineWidth', 2);
    quiver3(link_vecs(1,i), link_vecs(2,i), link_vecs(3,i),...
            J_com{i}(1,2), J_com{i}(2,2), J_com{i}(3,2),'y', 'LineWidth', 2);
    quiver3(link_vecs(1,i), link_vecs(2,i), link_vecs(3,i),...
            J_com{i}(1,3), J_com{i}(2,3), J_com{i}(3,3),'b', 'LineWidth', 2);
end
% for i = 1:numel(J_bod)
%     % one arrow for ea. joint contribution
%     quiver3(link_vecs(1,i), link_vecs(2,i), link_vecs(3,i),...
%             J_bod{i}(1,1), J_bod{i}(2,1), J_bod{i}(3,1),'r', 'LineWidth', 2);
%     quiver3(link_vecs(1,i), link_vecs(2,i), link_vecs(3,i),...
%             J_bod{i}(1,2), J_bod{i}(2,2), J_bod{i}(3,2),'y', 'LineWidth', 2);
%     quiver3(link_vecs(1,i), link_vecs(2,i), link_vecs(3,i),...
%             J_bod{i}(1,3), J_bod{i}(2,3), J_bod{i}(3,3),'b', 'LineWidth', 2);
% end
