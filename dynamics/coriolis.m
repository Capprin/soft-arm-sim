% computes coriolis term
function C = coriolis(dM_fun, q, q_dot)
    % get partials wrt configs
    dM = dM_fun(q);
    % do calc for each partial, adding on ea. iterations
    C = zeros(size(q,1));
    for i = 1:numel(dM)
        c1 = dM{i}*q_dot * q_dot;
        c2 = 1/2 * q_dot' * dM{i} * q_dot;
        C = C + c1 - c2;
    end
end