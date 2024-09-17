clc
clear

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

addpath(genpath("../../lib/utils"))
addpath(genpath("../../lib/charts_functions"))
addpath(genpath("../functions/plots"))

% Run some scripts to load the simulation parameters
run bluerov2_simulation_parameters
run bluerov2_models

%===================================================================================================
% Reference model for trajectory generation section
% According to Fossen 2021, p. 337, section 12.1.1 References Models for Trajectory Generation
% The reference model is given by the following transfer function:
%  							wn^3	
% H(s) = -----------------------------------------------------
%        s^3 + (2*ksi + 1)*wn*s^2 + (2*ksi + 1)*wn^2*s + wn^3
% Which is second order system cascaded with a first order system
% The state space representation of the reference model is given by:
%===================================================================================================
state_vector_size = size(nominal_model.discrete_state_space.Ad, 1);

bandwidth_reduction_factor.G_x_to_u = 2.0;
bandwidth_reduction_factor.G_y_to_v = 5.0;
bandwidth_reduction_factor.G_z_to_w = 3.0;
bandwidth_reduction_factor.G_n_to_r = 1.0;
ksi = 1/sqrt(2);

nominal_model.G_x_to_u.wn = bandwidth(nominal_model.tf.G_x_to_u)*bandwidth_reduction_factor.G_x_to_u;
nominal_model.G_y_to_v.wn = bandwidth(nominal_model.tf.G_y_to_v)*bandwidth_reduction_factor.G_y_to_v;
nominal_model.G_z_to_w.wn = bandwidth(nominal_model.tf.G_z_to_w)*bandwidth_reduction_factor.G_z_to_w;
nominal_model.G_n_to_r.wn = bandwidth(nominal_model.tf.G_n_to_r)*bandwidth_reduction_factor.G_n_to_r;

[nominal_model.pos_ref_tf.x, nominal_model.vel_ref_tf.x] = getDiscreteRefenceModelTf(ksi, nominal_model.G_x_to_u.wn, sampling_period);
[nominal_model.pos_ref_tf.y, nominal_model.vel_ref_tf.y] = getDiscreteRefenceModelTf(ksi, nominal_model.G_y_to_v.wn, sampling_period);
[nominal_model.pos_ref_tf.z, nominal_model.vel_ref_tf.z] = getDiscreteRefenceModelTf(ksi, nominal_model.G_z_to_w.wn, sampling_period);
[nominal_model.pos_ref_tf.psi, nominal_model.vel_ref_tf.psi] = getDiscreteRefenceModelTf(ksi, nominal_model.G_n_to_r.wn, sampling_period);

square_size_in_meters = 1;
linear_nav_vel_in_si = 0.1;
ang_vel_in_si = deg2rad(90)/5.0;  % 90 degrees in 5 seconds
is_to_plot_time_labels = false;

waypoints = {
	{[0 0 5   0  ],  0};
	{[1 0 5   0  ], 10};
	{[1 0 5 -pi/2], 15};
	{[1 0 2 -pi/2], 45};
	{[2 0 2 -pi/2], 55};
	{[2 0 5 -pi/2], 85};
	{[3 0 5 -pi/2], 95};
};

total_time = waypoints{end}{2};
t = 0:sampling_period:(total_time-sampling_period);
desired.trajectory = zeros(size(t, 2), state_vector_size);
desired.ned_velocity = zeros(size(t, 2), state_vector_size);

for i=2:length(waypoints)
	p0 = waypoints{i-1}{1};
	pf = waypoints{i}{1};
	t0 = waypoints{i-1}{2};
	tf = waypoints{i}{2};
	[desired.trajectory, desired.ned_velocity] = make_segment(t, t0, tf, p0, pf, desired.trajectory, desired.ned_velocity, 'lspb');
end

desired.pose = desired.trajectory;
desired.line_spec = 'r';
desired.line_width = 1.5;

desired.body_fixed_vel(:, 1) = zeros(size(desired.ned_velocity, 1), 1);
desired.body_fixed_vel(:, 2) = zeros(size(desired.ned_velocity, 1), 1);
desired.body_fixed_vel(:, 3) = desired.ned_velocity(:, 3);
desired.body_fixed_vel(:, 4) = desired.ned_velocity(:, 4);

x_dot = desired.ned_velocity(:, 1);
y_dot = desired.ned_velocity(:, 2);
yaw_d = desired.trajectory(:, 4);

for i = 1:length(x_dot)
	desired.body_fixed_vel(i, 1) = (x_dot(i) * cos(-yaw_d(i))) - (y_dot(i) * sin(-yaw_d(i)));
	desired.body_fixed_vel(i, 2) = (x_dot(i) * sin(-yaw_d(i))) + (y_dot(i) * cos(-yaw_d(i)));
end

figure("Name", "desired-3d-path")
plot_3d_path(desired)

figure("Name", "desired-trajectory")
desired_trajectory_args.y_labels = {'x [m]', 'y [m]', 'z [m]', '\psi [rad]'};
desired_trajectory_args.y_min_offset = 0.1;
desired_trajectory_args.y_max_offset = 0.1;
plot_per_dof_values(t, desired_trajectory_args, desired.trajectory)

figure("Name", "pose-and-ned-velocities");
plot_pose_and_ned_velocity(t, desired.trajectory, desired.ned_velocity)

figure("Name", "velocities-in-ned-frame")
desired_ned_vel_args.y_labels = {'$\dot{x}$ [m/s]', '$\dot{y}$ [m/s]', '$\dot{z}$ [m/s]', '$\dot{\psi}$ [rad/s]'};
desired_ned_vel_args.y_min_offset = 0.1;
desired_ned_vel_args.y_max_offset = 0.1;
plot_per_dof_values(t, desired_ned_vel_args, desired.ned_velocity)

figure("Name", "velocities-in-body-fixed-frame")
desired_body_fixed_vel_args.y_labels = {'u [m/s]', 'v [m/s]', 'w [m/s]', 'r [rad/s]'};
desired_body_fixed_vel_args.y_min_offset = 0.1;
desired_body_fixed_vel_args.y_max_offset = 0.1;
plot_per_dof_values(t, desired_body_fixed_vel_args, desired.body_fixed_vel)

function s = create_s()
	s = tf('s');
end

function [G_pos, G_vel] = getDiscreteRefenceModelTf(ksi, wn, sampling_period)
	s = create_s();
	T = 1/wn;
	G = (wn^2)/((1+T*s)*(s^2 + 2*ksi*wn*s + wn^2));
	G_pos = c2d(G, sampling_period, 'tustin');
	G_vel = c2d(s*G, sampling_period, 'tustin');  % s*G is used to get the velocity reference model
end

function plot_two_arrays_in_same_chart(t, array_with_left_label, array_with_right_label, label1, label2)
	yyaxis left
	title(['$' label1 '$ and $' label2 '$ in NED frame'], 'Interpreter', 'latex');
	plot(t, array_with_left_label, '-r', 'LineWidth', 1.5)
	ylim([(min(array_with_left_label) - 0.1) (max(array_with_left_label) + 0.1)])
	ylabel([label1 '(t) [m]'])
	ax = gca;
	ax.YColor = 'r';

	yyaxis right
	plot(t, array_with_right_label, '-b', 'LineWidth', 1.5)
	ylabel(['$\dot{' label1 '}(t)$ [$ms^{-1}$]'], 'Interpreter', 'latex')
	ylim([(min(array_with_right_label) - 0.1) (max(array_with_right_label) + 0.1)])
	xlim([min(t) max(t)])
	ax = gca;
	ax.YColor = 'b';

	legend({['$' label1 '$'], ['$\dot{' label1 '}$']}, 'Interpreter', 'latex')
	grid on
end

function plot_pose_and_ned_velocity(t, pose, velocity)
	x = pose(:, 1);
	y = pose(:, 2);
	z = pose(:, 3);
	yaw = pose(:, 4);

	x_dot = velocity(:, 1);
	y_dot = velocity(:, 2);
	z_dot = velocity(:, 3);
	yaw_dot = velocity(:, 4);

	subplot(4, 1, 1)
	plot_two_arrays_in_same_chart(t, x, x_dot, 'x', '\dot{x}');
	subplot(4, 1, 2)
	plot_two_arrays_in_same_chart(t, y, y_dot, 'y', '\dot{y}');
	subplot(4, 1, 3)
	plot_two_arrays_in_same_chart(t, z, z_dot, 'z', '\dot{z}');
	subplot(4, 1, 4)
	plot_two_arrays_in_same_chart(t, yaw, yaw_dot, '\psi', '\dot{\psi}');

	xlabel('Time [s]')
end

function [pose, pose_deriv] = make_segment(t, t0, tf, p0, pf, pose, pose_deriv, traj_gen)
% MAKE_SEGMENT Generates a segment of a trajectory
%
%   [pose, pose_deriv] = MAKE_SEGMENT(t, t0, tf, p0, pf, pose, pose_deriv, traj_gen)
%   generates a segment of a trajectory from an initial position p0 to a
%   final position pf using a specified trajectory generation method. The
%   function updates the pose and pose_deriv arrays with the new segment
%   and returns the updated arrays.
%
%   Inputs:
%       t - Time vector
%       t0 - Initial time for the segment
%       tf - Final time for the segment
%       p0 - Initial position for the segment
%       pf - Final position for the segment
%       pose - Array to store the positions of the trajectory
%       pose_deriv - Array to store the velocities of the trajectory
%       traj_gen - Trajectory generation method ('lspb' or 'tpoly')
%
%   Outputs:
%       pose - Updated array with the positions of the trajectory
%       pose_deriv - Updated array with the velocities of the trajectory

    % Calculate the indices for the time range
    idx = find(t >= t0 & t <= tf);

    % Initialize arrays for position, velocity, and acceleration
    s = zeros(length(idx), length(p0));
    sd = zeros(length(idx), length(p0));
    sdd = zeros(length(idx), length(p0));

    % Select the trajectory generation function
    if traj_gen == "lspb"
        traj_gen_function = @lspb;
    elseif traj_gen == "tpoly"
        traj_gen_function = @tpoly;
    end

    % Generate the trajectory for each dimension
    for j = 1:length(p0)
        [s(:, j), sd(:, j), sdd(:, j)] = traj_gen_function(p0(j), pf(j), t(idx) - t0);
        for i = 1:length(idx)
            pose(idx(i), j) = s(i, j);
            pose_deriv(idx(i), j) = sd(i, j);
        end
    end
end
