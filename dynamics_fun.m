% handles dynamics for ODE45 call
% "stacks" refer to [\beta; \dot{beta}; ...] vectors
function d_beta_stack = dynamics_fun(arm, control, beta_stack, t)
    %% setup
    n_params = numel(arm.geometry.joints);
    % get control signal
    alpha_stack = control.control_fun(arm, control, beta_stack, t);
    % generate state vectors
    q = [alpha_stack(1:n_params); beta_stack(1:n_params)];
    q_dot = [alpha_stack((n_params+1):2*n_params);...
             beta_stack((n_params+1):2*n_params)];

    %% generate constituent quantities
    % coriolis
    C = coriolis(arm.dM_fun, q, q_dot);
    % potential energy
    K_mat = [zeros(n_params) zeros(n_params); zeros(n_params) diag(arm.k)];
    gamma = 9.8; %gravity
    PE = potential_energy(K_mat, gamma, arm.masses, arm.J_links_fun(q), q);

    %% construct EOM
    % mass matrix manipulation
    M = arm.M_fun(q);
    M_alpha = M(:,1:n_params);
    M_beta = M(:,(n_params+1):end);
    A = [[eye(n_params); zeros(n_params)], -M_beta];
    % eom; produces motor torques and spring accels
    alpha_ddot = alpha_stack((2*n_params+1):end);
    res_stack = A\(C + PE + M_alpha*alpha_ddot); %inv(A)*(...) 

    %% generate output
    d_beta_stack = [beta_stack((n_params+1):2*n_params);...%beta vel
                    res_stack(4:6);res_stack(1:3)];%originally [alpha_torque;beta_accel]
    d_beta_stack = double(d_beta_stack);
end