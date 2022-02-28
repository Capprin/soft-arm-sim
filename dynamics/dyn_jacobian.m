% produces Jacobian for fully dynamic arm (including alpha, beta)
function J_cell = dyn_jacobian(J_fun, q)
    % get J based on total angle
    half = length(q)/2;
    joint_angles = q(1:half) + q(half:end);
    J_all = J_fun(joint_angles);
    % stack matrices for contributions from alpha, beta
    J_cell = cell(size(J_all));
    for i = 1:numel(J_cell)
        J_cell{i} = [J_all{i} J_all{i}];
    end
end