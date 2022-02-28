% SE(3) related functions and definitions
% assumes zyx rotation matrix
function group = SE3
    % dimensionality
    group.dim = 6;
    % trans., rot. matrices in SE(3)
    group.Rx = @(t) [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];
    group.Ry = @(t) [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];
    group.Rz = @(t) [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];
    group.R = @(t_vec) group.Rz(t_vec(3)) *...
                       group.Ry(t_vec(2)) *...
                       group.Rx(t_vec(1));
    group.g_mat = @(g_vec) [group.R(g_vec(4:end)) g_vec(1:3)'; 0 0 0 1];
    % velocity (Lie algebra) matrix repr.
    group.g_circ = @(g_dot) [0 -g_dot(6) g_dot(5) g_dot(1);...
                            g_dot(6) 0 -g_dot(4) g_dot(2);...
                            -g_dot(5) g_dot(4) 0 g_dot(3);...
                            0 0 0 0];
    % matrix to vector
    group.g_vec = @(g_mat) [g_mat(1,4), g_mat(2,4), g_mat(3,4), ...
                            rotm2eul(g_mat(1:3,1:3),'zyx')]';
end