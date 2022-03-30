sampling_period = 60e-3;
sampling_period_integration_step_ratio = 100;
global integration_step

integration_step = sampling_period/sampling_period_integration_step_ratio;

prediction_horizon = 10;  % Prediction Horizon
control_horizon = 10;  % Control Horizon
r = 1;    % Reference error weight matrix R = r*I
q = 1;    % Control effort weight matrix Q = q*I

