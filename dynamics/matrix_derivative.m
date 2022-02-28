% computes partial derivative of a tensor (mass) wrt configuration
function dM_fun = matrix_derivative(M_fun, n_params)
    % set up symbolic logic
    q = sym('q', [n_params 1]);
    assume(q, 'real');
    % evaluate mass matrix symbolically, compute derivatives
    M_sym = M_fun(q);
    dM_vec = jacobian(M_sym(:), q);
    % cast as matlab function, producing matrices
    dM_cols_fun = matlabFunction(dM_vec, 'var', {configuration});
    dM_fun = @(q_vec) cols_to_mat_cells(dM_cols_fun(q_vec), n_params);
end

% helper function, producing matrices
function mat_cells = cols_to_mat_cells(cols, n_params)
    mat_cells = cell(size(cols,2));
    for i = 1:numel(mat_cells)
        mat_cells{i} = reshape(cols(:,i), n_params, n_params);
    end
end