% manages control logic for dynamic arm
% add P/D control and/or torque maxima here
function alpha_stack = control_fun(arm, control, beta_stack, t)
    % get control signal
    alpha_mat = control.alpha(t)'; %transpose for stack ordering
    alpha_stack = alpha_mat(:); % pos, vel, accel
    % do P/D control for acceleration term (goal: make beta small)
    n_params = size(control.gains,2);
    pos_accel = control.gains(1,:)' .* beta_stack(1:n_params);
    dev_accel = control.gains(2,:)' .* beta_stack((n_params+1):2*n_params);
    alpha_stack((2*n_params+1):end) = pos_accel + dev_accel;
end