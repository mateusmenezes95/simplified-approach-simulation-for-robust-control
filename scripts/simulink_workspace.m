addpath(genpath("../lib")) % Add lib path to Octave script file search paths
addpath(genpath("../lib/mpc_functions")) % Add lib path to Octave script file search paths

run robot_model

sampling_period = 60e-3;

prediction_horizon = 4;  % Prediction Horizon
control_horizon = 4;  % Control Horizon
r = 1;    % Reference error weight matrix R = r*I
q = 10;    % Control effort weight matrix Q = q*I

robot_vel = 0.3;

square_trajectory = generate_square_trajectory(1, robot_vel, sampling_period);
%square_trajectory = [ones(1,waypoints_qty); zeros(1,waypoints_qty); zeros(1,waypoints_qty)];
waypoints_qty = length(square_trajectory);

%=======================================================================================================================
% MPC Initialization
%=======================================================================================================================
[Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
[Kw, Kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);
