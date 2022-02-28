% handles dynamics for ODE45 call
% "stacks" refer to [\beta; \dot{beta}; ...] vectors
function d_beta_stack = dynamics_fun(arm, control, beta_stack, t)
    %% setup
    n_params = numel(arm.geometry.joints);
    % get control signal
    alpha_stack = control.control_fun(control, beta_stack, t);
    % generate state vectors
    q = [alpha_stack(1:n_params); beta_stack(1:n_params)];
    q_dot = [alpha_stack((n_params+1):2*n_params);...
             beta_stack((n_params+1):2*n_params)];

    %% generate constituent functions
    % Jacobians
    J_links_kin = @(alpha) link_jacobian(arm.group, arm.geometry, arm.coms, alpha);
    J_links_fun = @(q_vec) dyn_jacobian(J_links_kin, q_vec);
    % mass matrix
    M_fun = @(q_vec) mass_matrix(arm.masses, arm.inertias, J_links_fun(q_vec));
    dM_fun = matrix_derivative(M_fun, n_params);
    % coriolis
    C = coriolis(dM_fun, q, q_dot);
    % potential energy
    K_mat = [zeros(n_params) zeros(n_params); zeros(n_params) diag(arm.k)];
    gamma = 9.8; %gravity
    PE = potential_energy(K_mat, gamma, arm.masses, J_links_fun(q));

    %% construct EOM
    % mass matrix manipulation
    M = M_fun(q);
    M_alpha = M(:,1:n_params);
    M_beta = M(:,(n_params+1):end);
    A = [[eye(n_params); zeros(n_params)]; -M_beta];
    % eom; produces motor torques and spring accels
    alpha_ddot = alpha_stack((2*n_params+1):end);
    res_stack = A\(C + PE + M_alpha*alpha_ddot); %inv(A)*(...)

    %% generate output
    d_beta_stack = [beta_stack((2*n_params+1):3*n_params);...%beta vel
                    flipud(res_stack);...%originally [alpha_torque;beta_accel]
                    zeros(n_params,1)];%set rotatum (time derivative) to zero
end