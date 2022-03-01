% computes coriolis term
function C = coriolis(dM_fun, q, q_dot)
    % get partials wrt configs
    dM = dM_fun(q);
    % do calc for each partial, adding on ea. iterations
    c1 = zeros(size(q_dot));
    for i = 1:numel(c1)
        c1 = c1 + dM{i} * q_dot(i) * q_dot;
    end
    c2 = zeros(size(q_dot));
    for i = 1:numel(c2)
        c2(i) = 1/2 * q_dot' * dM{i} * q_dot;
    end
    C = c1 - c2;
end