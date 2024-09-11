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

time_segment_for_xy_plane = [
	0 10;		 % Line:1 - Navigating in the +x direction
	10 15;   % Line:2 - Turning 90 degrees in counterclockwise direction
	15 25;   % Line:3 - Navigating in the +y direction
	25 30;   % Line:4 - Turning 90 degrees in counterclockwise direction
	30 40;   % Line:5 - Navigating in the -x direction
	40 45;   % Line:6 - Turning 90 degrees in counterclockwise direction
	45 55;   % Line:7 - Navigating in the -y direction
	55 60;   % Line:8 - Turning 90 degrees in counterclockwise direction
];

time_segment_for_z = [
	0 15 1;		 % Line:1 - Depth = 1m during 1 second
	16 17 2;		 % Line:2 - Depth = 2m during 1 second
	17 23 2;		 % Line:3 - Depth = 2m during 6 seconds
	23 24 1;		 % Line:4 - Depth = 1m during 1 second
];

square_size_in_meters = 1;
linear_nav_vel_in_si = 0.1;
ang_vel_in_si = deg2rad(90)/5.0;  % 90 degrees in 5 seconds
is_to_plot_time_labels = true;

total_time = time_segment_for_xy_plane(end, 2);
t = 0:sampling_period:total_time;

x = zeros(size(t));
y = zeros(size(t)); % Initialize x as a vector of zeros with the same size as t
z = ones(size(t));
psi = zeros(size(t));

t0 = time_segment_for_xy_plane(1, 1);
tf = time_segment_for_xy_plane(1, 2);
x(t >= t0 & t < tf) = linear_nav_vel_in_si * (t(t >= t0 & t < tf) - t0); % Navigating in the +x direction
y(t >= t0 & t < tf) = 0;
psi(t >= t0 & t < tf) = 0;

t0 = time_segment_for_xy_plane(2, 1);
tf = time_segment_for_xy_plane(2, 2);
x(t >= t0 & t < tf) = square_size_in_meters;
y(t >= t0 & t < tf) = 0;
psi(t >= t0 & t < tf) = ang_vel_in_si * (t(t >= t0 & t < tf) - t0);

t0 = time_segment_for_xy_plane(3, 1);
tf = time_segment_for_xy_plane(3, 2);
x(t >= t0 & t < tf) = square_size_in_meters;
y(t >= t0 & t < tf) = linear_nav_vel_in_si * (t(t >= t0 & t < tf) - t0); % Navigating in the +y direction
psi(t >= t0 & t < tf) = deg2rad(90);

t0 = time_segment_for_xy_plane(4, 1);
tf = time_segment_for_xy_plane(4, 2);
x(t >= t0 & t < tf) = square_size_in_meters;
y(t >= t0 & t < tf) = square_size_in_meters;
psi(t >= t0 & t < tf) = ang_vel_in_si * (t(t >= t0 & t < tf) - t0) + deg2rad(90);

t0 = time_segment_for_xy_plane(5, 1);
tf = time_segment_for_xy_plane(5, 2);
x(t >= t0 & t < tf) = square_size_in_meters - linear_nav_vel_in_si * (t(t >= t0 & t < tf) - t0); % Navigating in the -x direction
y(t >= t0 & t < tf) = square_size_in_meters;
psi(t >= t0 & t < tf) = deg2rad(180);

t0 = time_segment_for_xy_plane(6, 1);
tf = time_segment_for_xy_plane(6, 2);
x(t >= t0 & t < tf) = 0;
y(t >= t0 & t < tf) = square_size_in_meters;
psi(t >= t0 & t < tf) = ang_vel_in_si * (t(t >= t0 & t < tf) - t0) + deg2rad(180);

t0 = time_segment_for_xy_plane(7, 1);
tf = time_segment_for_xy_plane(7, 2);
x(t >= t0 & t < tf) = 0;
y(t >= t0 & t < tf) = square_size_in_meters - linear_nav_vel_in_si * (t(t >= t0 & t < tf) - t0); % Navigating in the -y direction
psi(t >= t0 & t < tf) = deg2rad(270);

t0 = time_segment_for_xy_plane(8, 1);
tf = time_segment_for_xy_plane(8, 2);
x(t >= t0 & t < tf) = 0;
y(t >= t0 & t < tf) = 0;
psi(t >= t0 & t < tf) = ang_vel_in_si * (t(t >= t0 & t < tf) - t0) + deg2rad(270);

t0 = time_segment_for_z(1, 1);
tf = time_segment_for_z(1, 2);
depth = time_segment_for_z(1, 3);
z(t >= t0 & t < tf) = depth;
last_depth = depth;

for i = 2:size(time_segment_for_z, 1)
	t0 = time_segment_for_z(i, 1);
	tf = time_segment_for_z(i, 2);
	depth = time_segment_for_z(i, 3);
	z(t >= t0 & t < tf) = last_depth + (depth - last_depth) * (t(t >= t0 & t < tf) - t0);
	last_depth = depth;
end

figure("Name", "bluerov-3d-trajectory")
plot_3d_robot_path(x, y, z, 'r', 1.5)

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

x_dot = lsim(nominal_model.vel_ref_tf.x, x, t);
y_dot = lsim(nominal_model.vel_ref_tf.y, y, t);
z_dot = lsim(nominal_model.vel_ref_tf.z, z, t);
psi_dot = lsim(nominal_model.vel_ref_tf.psi, psi, t);

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
	xlabel('x [m]')
	ylabel('y [m]')
	zlabel('z [m]')
end
