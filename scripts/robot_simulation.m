addpath(genpath("../lib"))
addpath(genpath("../lib/mpc_functions"))

run simulation_parameters
run robot_model

robot_vel = 0.3;

square_trajectory = generate_square_trajectory(1, robot_vel, sampling_period);
waypoints_qty = length(square_trajectory);

r = 10000;
q = 900;

%=======================================================================================================================
% MPC Initialization
%=======================================================================================================================
[Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
[Kw, Kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);

Nmax = 4;

print_section_description("Running Robot Simulation on Simulink...")
sim_out = sim('./simulink/robot.slx');
print_section_description("Robot Simulation Finished!")

% xr = robot_pose(1,:);
% yr = robot_pose(2,:);

% figure(1)
% plot_robot_states(t,continous_x,robot_discrete_model.stname);
% figure(2)
% plot_robot_velocities(t,wheels_linear_vel);
% figure(3)
% plot_robot_trajectory(xr, yr)
% figure(4)
% plot_control_signals(t,continous_u);
