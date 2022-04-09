addpath(genpath("./lib"))
addpath(genpath("./lib/mpc_functions"))
addpath(genpath("./lib/chart_functions"))

run simulation_parameters
run robot_model

robot_vel = 0.05;
square_trajectory = generate_square_trajectory(1, robot_vel, sampling_period);
x_trajectory = square_trajectory(1,:);
y_trajectory = square_trajectory(2,:);
waypoints_qty = length(square_trajectory);

sim_time = 90;  % Choosen due results inpection

%==============================================================================
% MPC Initialization
%==============================================================================
r = 10000;
q = 531;
[Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
[Kw, Kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);

Nmax = 1;

%==============================================================================
% Run Simulation
%==============================================================================
print_section_description("Running Robot Simulation on Simulink...")
sim_out = sim('./simulink/robot.slx');
print_section_description("Robot Simulation Finished!")

%==============================================================================
% Plot results
%==============================================================================
font_size = 10;
line_thickness = 1;
y_axis_limits_offset = 0.2;
figure_idx = 1;
plot_step_response = false;
%==============================================================================

%==============================================================================
% Plot trajectories
%==============================================================================
figure(figure_idx)
plot_robot_trajectory(x_trajectory, y_trajectory, font_size, line_thickness, '--')
xlim([-0.2 1.2]);
ylim([-0.2 1.2]);
hold on
plot_robot_trajectory(sim_out.x_pose_without_delay, sim_out.y_pose_without_delay, font_size, line_thickness, '-')
plot_robot_trajectory(sim_out.x_pose_with_delay, sim_out.y_pose_with_delay, font_size, line_thickness, '-')
legend({'ref', 'Nmax = 0', ['Nmax = ' num2str(Nmax)]})
%==============================================================================

figure_idx = figure_idx + 1;

%==============================================================================
% Plot robot poses
%==============================================================================
figure(figure_idx)
robot_pose_without_delay = [sim_out.x_pose_without_delay, sim_out.y_pose_without_delay sim_out.theta_pose_without_delay];
robot_pose_with_delay = [sim_out.x_pose_with_delay, sim_out.y_pose_with_delay sim_out.theta_pose_with_delay];
plot_robot_pose(sim_out.sim_time_sampled, robot_pose_without_delay, line_thickness)
plot_robot_pose(sim_out.sim_time_sampled, robot_pose_with_delay, line_thickness)
legend({'Nmax = 0', ['Nmax = ' num2str(Nmax)]})
%==============================================================================

figure_idx = figure_idx + 1;

%==============================================================================
% Plot control signals
%==============================================================================
figure(figure_idx)
u_with_delay= [sim_out.u1_with_delay sim_out.u2_with_delay sim_out.u3_with_delay];
u_without_delay= [sim_out.u1_without_delay sim_out.u2_without_delay sim_out.u3_without_delay];
plot_control_signals(sim_out.sim_time_sampled, u_with_delay, font_size, line_thickness)
plot_control_signals(sim_out.sim_time_sampled, u_without_delay, font_size, line_thickness)
legend({'Nmax = 0', ['Nmax = ' num2str(Nmax)]})
%==============================================================================

figure_idx = figure_idx + 1;

%==============================================================================
% Plot step responses
%==============================================================================
if plot_step_response
    figure(figure_idx)
    state_step = [sim_out.step_v sim_out.step_vn sim_out.step_w];
    state_step_response = [sim_out.step_reponse_v sim_out.step_reponse_vn sim_out.step_reponse_w];
    plot_robot_states(sim_out.sim_time_sampled, state_step, {'v', 'vn', 'w'}, font_size, line_thickness)
    plot_robot_states(sim_out.sim_time_sampled, state_step_response, {'v', 'vn', 'w'}, font_size, line_thickness)
    legend({'referencia', 'resposta'})
    
    figure_idx = figure_idx + 1;
    
    figure(figure_idx)
    u_step_response = [sim_out.step_reponse_u1 sim_out.step_reponse_u2 sim_out.step_reponse_u2];
    plot_control_signals(sim_out.sim_time_sampled, u_step_response, font_size, line_thickness)
end
