clc
clear

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

addpath(genpath("../../lib/utils"))

% Run some scripts to load the simulation parameters
run bluerov2_simulation_parameters
run bluerov2_models

%===================================================================================================
% Reference model for trajectory generation section
% According to Fossen 2021, p. 337, section 12.1.1 References Models for Trajectory Generation
% The reference model is given by the following transfer function:
%  																wn^3	
% H(s) = -----------------------------------------------------
%        s^3 + (2*ksi + 1)*wn*s^2 + (2*ksi + 1)*wn^2*s + wn^3
% Which is second order system cascaded with a first order system
% The state space representation of the reference model is given by:
%===================================================================================================
state_vector_size = size(nominal_model.discrete_state_space.Ad, 1);

bandwidth_reduction_factor = 0.9;
ksi = 1/sqrt(2);

nominal_model.G_x_to_u.wn = bandwidth(nominal_model.tf.G_x_to_u)*bandwidth_reduction_factor;
nominal_model.G_y_to_v.wn = bandwidth(nominal_model.tf.G_y_to_v)*3;
nominal_model.G_z_to_w.wn = bandwidth(nominal_model.tf.G_z_to_w)*bandwidth_reduction_factor;
nominal_model.G_n_to_r.wn = bandwidth(nominal_model.tf.G_n_to_r)*bandwidth_reduction_factor;

[nominal_model.pos_ref_tf.x, nominal_model.vel_ref_tf.x] = getDiscreteRefenceModelTf(ksi, nominal_model.G_x_to_u.wn, sampling_period);
[nominal_model.pos_ref_tf.y, nominal_model.vel_ref_tf.y] = getDiscreteRefenceModelTf(ksi, nominal_model.G_y_to_v.wn, sampling_period);
[nominal_model.pos_ref_tf.z, nominal_model.vel_ref_tf.z] = getDiscreteRefenceModelTf(ksi, nominal_model.G_z_to_w.wn, sampling_period);
[nominal_model.pos_ref_tf.psi, nominal_model.vel_ref_tf.psi] = getDiscreteRefenceModelTf(ksi, nominal_model.G_n_to_r.wn, sampling_period);

square_size_in_meters = 1;
nav_vel_in_meter_per_second = 0.1;
total_time = (square_size_in_meters*4)/nav_vel_in_meter_per_second;
t = 0:sampling_period:total_time;

x = zeros(size(t));
x(t < 0) = 0;
x(0 <= t & t < 10) = nav_vel_in_meter_per_second * t(0 <= t & t < 10);
x(10 <= t & t < 20) = 1;
x(20 <= t & t < 30) = 1 - nav_vel_in_meter_per_second * (t(20 <= t & t < 30) - 20);
x(t >= 30) = 0;

y = zeros(size(t)); % Initialize x as a vector of zeros with the same size as t
% Define the piecewise function
y(t < 10) = 0;
y(10 <= t & t < 20) = nav_vel_in_meter_per_second * (t(10 <= t & t < 20) - 10);
y(20 <= t & t < 30) = 1;
y(30 <= t & t < 40) = 1 - nav_vel_in_meter_per_second * (t(30 <= t & t < 40) - 30);
y(t >= 40) = 0;

z = zeros(size(t));

psi = zeros(size(t));
psi(10 <= t & t < 20) = deg2rad(90);
psi(20 <= t & t < 30) = deg2rad(180);
psi(30 <= t & t <= 40) = deg2rad(270);

x_dot = lsim(nominal_model.vel_ref_tf.x, x, t);
y_dot = lsim(nominal_model.vel_ref_tf.y, y, t);
z_dot = lsim(nominal_model.vel_ref_tf.z, z, t);
psi_dot = lsim(nominal_model.vel_ref_tf.psi, psi, t);

fig = figure;
left_color = [1 0 0];  % Red
right_color = [0 0 1];  % Blue
set(fig,'defaultAxesColorOrder',[left_color; right_color]);

subplot(4, 1, 1)
plot_states(t, x, x_dot, 'x', '\dot{x}');
subplot(4, 1, 2)
plot_states(t, y, y_dot, 'y', '\dot{y}');
subplot(4, 1, 3)
plot_states(t, z, z_dot, 'z', '\dot{z}');
subplot(4, 1, 4)
plot_states(t, psi, psi_dot, '\psi', '\dot{\psi}');

x_dot_rotated = zeros(size(x_dot));
y_dot_rotated = zeros(size(y_dot));

for i = 1:length(x_dot)
	x_dot_rotated(i) = (x_dot(i) * cos(-psi(i))) - (y_dot(i) * sin(-psi(i)));
	y_dot_rotated(i) = (x_dot(i) * sin(-psi(i))) + (y_dot(i) * cos(-psi(i)));
end

figure
subplot(3, 1, 1)
plot(t, x_dot_rotated)
grid on
subplot(3, 1, 2)
plot(t, y_dot_rotated)
grid on
subplot(3, 1, 3)
plot(t, psi_dot)
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

function plot_states(t, pos_or_attitude, pos_or_attitude_dot, label1, label2)
	yyaxis left
	title(['$' label1 '$ and $' label2 '$ in NED frame'], 'Interpreter', 'latex');
	plot(t, pos_or_attitude)
	ylim([min(pos_or_attitude) max(pos_or_attitude)+0.1])
	ylabel([label1 '(t) [m]'])

	yyaxis right
	plot(t, pos_or_attitude_dot)
	ylabel(['$\dot{' label1 '}(t)$ [$ms^{-1}$]'], 'Interpreter', 'latex')
	ylim([min(pos_or_attitude_dot)-0.1 max(pos_or_attitude_dot)+0.1])

	legend({['$' label1 '$'], ['$\dot{' label1 '}$']}, 'Interpreter', 'latex')
	grid on
end
