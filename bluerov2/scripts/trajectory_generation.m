clc
clear

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

addpath(genpath("../../lib/utils"))
addpath(genpath("../../lib/charts_functions"))

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

total_time = 55;
t = 0:sampling_period:(total_time-sampling_period);
trajectory = zeros(size(t, 2), state_vector_size);
velocity = zeros(size(t, 2), state_vector_size);

waypoints = {
	{[0 0 5 0], 0};
	{[1 0 5 0], 10};
	{[1 0 5 -pi/2], 15};
	{[1 0 2 -pi/2], 25};
	{[2 0 2 -pi/2], 35};
	{[2 0 5 -pi/2], 45};
	{[3 0 5 -pi/2], 55};
};

for i=2:length(waypoints)
	p0 = waypoints{i-1}{1};
	pf = waypoints{i}{1};
	t0 = waypoints{i-1}{2};
	tf = waypoints{i}{2};
	[trajectory, velocity] = make_segment(t, t0, tf, p0, pf, trajectory, velocity);
end

x = trajectory(:, 1);
y = trajectory(:, 2);
z = trajectory(:, 3);
psi = trajectory(:, 4);

figure("Name", "bluerov-3d-trajectory")
plot_3d_robot_path(x', y', z', 'r', 1.5)

time_points = [10 15 25 30 40 45 55 60];
time_labels = {'t_1', 't_2', 't_3', 't_4', 't_5', 't_6', 't_7', 't_8'};

figure("Name", "Position and Attitude in NED frame")
subplot(4, 1, 1)
plot_states(t, x, 'x [m]')
plot_time_labels(time_points, time_labels, is_to_plot_time_labels)

subplot(4, 1, 2)
plot_states(t, y, 'y [m]')
plot_time_labels(time_points, time_labels, is_to_plot_time_labels)

subplot(4, 1, 3)
plot_states(t, z, 'z [m]')
plot_time_labels(time_points, time_labels, is_to_plot_time_labels)

subplot(4, 1, 4)
plot_states(t, rad2deg(psi), '\psi [deg]')
plot_time_labels(time_points, time_labels, is_to_plot_time_labels)

xlabel('Time [s]')

x_dot = velocity(:, 1);
y_dot = velocity(:, 2);
z_dot = velocity(:, 3);
psi_dot = velocity(:, 4);

fig = figure("Name", "Position and Velocity in NED frame");
left_color = [1 0 0];  % Red
right_color = [0 0 1];  % Blue
set(fig,'defaultAxesColorOrder',[left_color; right_color]);

subplot(4, 1, 1)
plot_pose_and_velocity(t, x, x_dot, 'x', '\dot{x}');
subplot(4, 1, 2)
plot_pose_and_velocity(t, y, y_dot, 'y', '\dot{y}');
subplot(4, 1, 3)
plot_pose_and_velocity(t, z, z_dot, 'z', '\dot{z}');
subplot(4, 1, 4)
plot_pose_and_velocity(t, psi, psi_dot, '\psi', '\dot{\psi}');

figure("Name", "Velocities in NED frame")
subplot(4, 1, 1)
plot(t, x_dot, 'r', 'LineWidth', 1.5)
ylabel('$\dot{x}$ [m/s]', 'Interpreter', 'latex')
grid on
subplot(4, 1, 2)
plot(t, y_dot, 'r', 'LineWidth', 1.5)
ylabel('$\dot{y}$ [m/s]', 'Interpreter', 'latex')
grid on
subplot(4, 1, 3)
plot(t, z_dot, 'r', 'LineWidth', 1.5)
ylabel('$\dot{z}$ [m/s]', 'Interpreter', 'latex')
grid on
subplot(4, 1, 4)
plot(t, psi_dot, 'r', 'LineWidth', 1.5)
xlabel('Tempo [s]')
ylabel('$\dot{\psi}$ [rad/s]', 'Interpreter', 'latex')
grid on

x_dot_rotated = zeros(size(x_dot));
y_dot_rotated = zeros(size(y_dot));
z_dot_rotated = z_dot;  % z_dot is the same in both frames because there is rotation only in the x-y plane
psi_filtered = lsim(nominal_model.pos_ref_tf.psi, psi, t);

for i = 1:length(x_dot)
	x_dot_rotated(i) = (x_dot(i) * cos(-psi(i))) - (y_dot(i) * sin(-psi(i)));
	y_dot_rotated(i) = (x_dot(i) * sin(-psi(i))) + (y_dot(i) * cos(-psi(i)));
end

figure("Name", "Velocities in body-fixed frame")
subplot(4, 1, 1)
plot(t, x_dot_rotated, 'r', 'LineWidth', 1.5)
ylabel('u [m/s]')
grid on
subplot(4, 1, 2)
plot(t, y_dot_rotated, 'r', 'LineWidth', 1.5)
ylabel('v [m/s]')
grid on
subplot(4, 1, 3)
plot(t, z_dot, 'r', 'LineWidth', 1.5)
ylabel('w [m/s]')
grid on
subplot(4, 1, 4)
plot(t, psi_dot, 'r', 'LineWidth', 1.5)
xlabel('Time [s]')
ylabel('r [rad/s]')
grid on

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

function plot_states(t, pos_or_attitude, ylabel_name)
	plot(t, pos_or_attitude, 'LineWidth', 1.5, 'Color', 'r')
	ylim([min(pos_or_attitude) max(pos_or_attitude)+0.1])
	ylabel(ylabel_name)
	grid on
end

function plot_pose_and_velocity(t, pos_or_attitude, pos_or_attitude_dot, label1, label2)
	yyaxis left
	title(['$' label1 '$ and $' label2 '$ in NED frame'], 'Interpreter', 'latex');
	plot(t, pos_or_attitude, 'LineWidth', 1.5)
	ylim([min(pos_or_attitude) max(pos_or_attitude)+0.1])
	ylabel([label1 '(t) [m]'])

	yyaxis right
	plot(t, pos_or_attitude_dot, 'LineWidth', 1.5)
	ylabel(['$\dot{' label1 '}(t)$ [$ms^{-1}$]'], 'Interpreter', 'latex')
	ylim([min(pos_or_attitude_dot)-0.1 max(pos_or_attitude_dot)+0.1])

	legend({['$' label1 '$'], ['$\dot{' label1 '}$']}, 'Interpreter', 'latex')
	grid on
end

function plot_time_labels(time_points, time_labels, is_to_plot_time_labels)
	if ~is_to_plot_time_labels
		return
	end

	hold on
	for i = 1:length(time_points)
		xline(time_points(i), '--k', time_labels{i}, 'LabelOrientation', 'horizontal', 'LabelVerticalAlignment', 'middle')
	end
	hold off
end

function plot_3d_robot_path(x, y, z, line_color, line_width)
	plot3(x(1,:), y(1,:), z(1,:), line_color, 'linewidth', line_width)
	[x_min, x_max] = get_axis_limits(x, 0.1);
	[y_min, y_max] = get_axis_limits(y, 0.1);
	[z_min, z_max] = get_axis_limits(z, 0.1);
	grid on
	axis([x_min, x_max, y_min, y_max, z_min, z_max])
	set(gca, 'ZDir', 'reverse')
	xlabel('x [m]')
	ylabel('y [m]')
	zlabel('z [m]')
end

function [pose, pose_deriv] = make_segment(t, t0, tf, p0, pf, pose, pose_deriv)
% MAKE_SEGMENT Generates a segment of a trajectory
%
%   [pose, pose_deriv, tf] = MAKE_SEGMENT(t, t0, p0, pf, vel, pose, pose_deriv)
%   generates a segment of a trajectory from an initial position p0 to a
%   final position pf with a given velocity vel. The function updates the
%   pose and pose_deriv arrays with the new segment and returns the updated
%   arrays along with the final time tf for the segment.
%
%   Inputs:
%       t - Time vector
%       t0 - Initial time for the segment
%       p0 - Initial position for the segment
%       pf - Final position for the segment
%       vel - Velocity array for the segment
%       pose - Array to store the positions of the trajectory
%       pose_deriv - Array to store the velocities of the trajectory
%
%   Outputs:
%       pose - Updated array with the positions of the trajectory
%       pose_deriv - Updated array with the velocities of the trajectory
%       tf - Final time for the segment

    % Calculate the final time for the segment
		vel = (pf - p0) ./ (tf - t0);
		idx = find(t >= t0 & t < tf);

		for i = 1:length(idx)
			pose(idx(i), :) = p0 + vel * (t(idx(i)) - t0);
			pose_deriv(idx(i), :) = vel;
		end
end
