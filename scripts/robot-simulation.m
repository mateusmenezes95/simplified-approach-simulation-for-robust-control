addpath(genpath("../lib")) % Add lib path to Octave script file search paths

run simulation-parameters
run util-functions
run robot-model
run charts-functions
run trajectory-functions
run mpc-functions

%=======================================================================================================================
% Simulation time parameters
%=======================================================================================================================
sim_time = 3;
step_begin = sampling_period;
step_begin_idx = floor(step_begin/sampling_period);
t = 0:integration_step:sim_time;
discrete_samples = floor(sim_time/sampling_period)

%=======================================================================================================================
% Reference computation
%=======================================================================================================================
discrete_y_ref = zeros(state_vector_size, discrete_samples + prediction_horizon);
discrete_y_ref(1,step_begin_idx:end) = ones(1, (length(discrete_samples)-step_begin_idx+1));

%=======================================================================================================================
% MPC Initialization
%=======================================================================================================================
[Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
[Kw, Kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, 100, 1, prediction_horizon, control_horizon);

%=======================================================================================================================
% Inital conditions of the plant
%=======================================================================================================================
continous_x0 = [0 0 0]';
discrete_x0 = continous_x0;
continous_u0 = [0 0 0]';
discrete_u0 = [0 0 0]';
discrete_delta_x0(:,1) = [0 0 0]';
y0 = [0 0 0]';
ksi0 = [discrete_delta_x0; y0];
robot_pose0 = [0 0 0]';

%=======================================================================================================================
% First control horizon computation
%=======================================================================================================================
k=1;
horizon_references = get_horizon_references(1, prediction_horizon, discrete_y_ref);
delta_u0 = Kw*horizon_references - Kmpc*ksi0;

[continous_x(:,1), wheels_linear_vel(:,1)] = robot(continous_x0, continous_u0);

for dt=1:length(t)
  if ((mod(dt,sampling_period_integration_step_ratio) == 0) || (dt == 1))
    discrete_x(:,k) = continous_x(:,dt);
    discrete_y(:,k) = robot_discrete_model.C*discrete_x(:,k);

    if (k == 1)
      delta_x(:,k) = discrete_x(:,k) - discrete_x0;
    else
      delta_x(:,k) = discrete_x(:,k) - discrete_x(:,k-1);
    endif

    horizon_references = get_horizon_references(k+1, prediction_horizon, discrete_y_ref);
    ksi(:,k) = [delta_x(:,k); discrete_y(:,k)];
    delta_u(:,k) = Kw*horizon_references - Kmpc*ksi(:,k);

    if (k == 1)
      discrete_u(:,k) = delta_u(1:state_vector_size,k) + discrete_u0;
    else
      discrete_u(:,k) = delta_u(1:state_vector_size,k) + discrete_u(:,k-1);
    endif

    ukk = discrete_u(:, k);

    k = k+1;
  endif

  continous_u(:,dt) = ukk;

  [continous_x(:,dt+1), wheels_linear_vel(:,dt)] = robot(continous_x(:,dt), continous_u(:,dt));

  if dt == 1
    robot_pose(:,dt) = compute_odometry(robot_pose0, continous_x(:,dt));
  else
    robot_pose(:,dt) = compute_odometry(robot_pose(:,dt-1), continous_x(:,dt));
  endif
endfor

continous_x = continous_x(:,1:end-1);

xr = robot_pose(1,:);
yr = robot_pose(2,:);

figure(1)
plot_robot_states(t,continous_x,robot_discrete_model.stname);
figure(2)
plot_robot_velocities(t,wheels_linear_vel);
figure(3)
plot_robot_trajectory(xr, yr)
figure(4)
plot_control_signals(t,continous_u);
