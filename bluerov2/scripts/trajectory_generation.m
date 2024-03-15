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

ksi = 1/sqrt(2);
delta_matrix = diag([ksi, ksi, ksi, ksi]);

bandwidth_reduction_factor = 0.9;
omega_matrix = diag([...
	bandwidth(nominal_model.tf.G_x_to_u)*bandwidth_reduction_factor, ...
	bandwidth(nominal_model.tf.G_y_to_v)*bandwidth_reduction_factor, ...
	bandwidth(nominal_model.tf.G_z_to_w)*bandwidth_reduction_factor, ...
	bandwidth(nominal_model.tf.G_n_to_r)*bandwidth_reduction_factor
]);

wn = bandwidth(nominal_model.tf.G_x_to_u)*bandwidth_reduction_factor;
T = 1/wn;
s = tf('s');
G = (wn^2)/((1+T*s)*(s^2 + 2*ksi*wn*s + wn^2));
Gd = c2d(G, sampling_period, 'tustin');

zero_matrix = zeros(state_vector_size, state_vector_size);
eye_matrix = eye(state_vector_size, state_vector_size);
reference_model_A = [zero_matrix eye_matrix zero_matrix; ...
												zero_matrix zero_matrix eye_matrix; ...
												-omega_matrix^3 -((2*delta_matrix + eye_matrix)*(omega_matrix^2)) -(2*delta_matrix + eye_matrix)*omega_matrix];
reference_model_B = [zero_matrix; zero_matrix; omega_matrix^3];

reference_model_C = [eye_matrix zero_matrix zero_matrix; ...
												zero_matrix zero_matrix zero_matrix; ...
												zero_matrix zero_matrix zero_matrix];

reference_model_ss = ss(reference_model_A, reference_model_B, reference_model_C, 0, sampling_period);

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

subplot(3, 1, 1)
% Plot the function
plot(t, x)
xlabel('Time')
ylabel('x(t)')
title('x(t) in NED frame')
ylim([0 1.1])
grid on

subplot(3, 1, 2)

% Plot the function
plot(t, y)
xlabel('Time')
ylabel('y(t)')
title('y(t) in NED frame')
ylim([0 1.1])
grid on

subplot(3, 1, 3)
plot(t, psi)
xlabel('Time')
ylabel('psi(t)')
title('psi(t) in NED frame')
ylim([0 deg2rad(360)])
grid on

u = [x' y' z' psi'];
