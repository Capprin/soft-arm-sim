% manages control logic for dynamic arm
% add P/D control and/or torque maxima here
function alpha_stack = control_fun(control, beta_stack, t)
    % to start, just relay control signal
    alpha_mat = control.alpha(t);
    alpha_stack = alpha_mat(:);
end