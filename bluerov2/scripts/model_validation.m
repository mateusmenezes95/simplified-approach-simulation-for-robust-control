clc
clear

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

addpath(genpath("."))
addpath(genpath("../functions/numerical_integration"))
addpath(genpath("../functions/matrices_getters"))
addpath(genpath("../../lib/utils"))

% Run some scripts to load the simulation parameters
run bluerov2_simulation_parameters
run bluerov2_models

nominal_model.gravity_vector = [0; 0; 2.5; 0];
non_linear_map_args.dynamic_model = nominal_model;

pwm_output_values = readmatrix('../datalogs/tests-on_04-23-2024/analysis-interval-01/rcou-data.csv');
ekf_output_values = readmatrix('../datalogs/tests-on_04-23-2024/analysis-interval-01/xkf1-data.csv');

time_index = 2;
thruster_index.vertical.right = 7;
thruster_index.vertical.left = 8;

velocity_index.north = 7;
velocity_index.east = 8;
velocity_index.down = 9;

orientation_index.roll = 4;
orientation_index.pitch = 5;
orientation_index.yaw = 6;

time_epochs = pwm_output_values(:, time_index);
time_from_start = (time_epochs - time_epochs(1))/1e6;

vertical_thruster_pwm_val.right = pwm_output_values(:, thruster_index.vertical.right);
vertical_thruster_pwm_val.left = pwm_output_values(:, thruster_index.vertical.left);

vertical_thruster_thrust_val.right = pwm_to_thrust(vertical_thruster_pwm_val.right, "cw");
vertical_thruster_thrust_val.left = pwm_to_thrust(vertical_thruster_pwm_val.left, "ccw");

total_vertical_thrust = vertical_thruster_thrust_val.right + vertical_thruster_thrust_val.left;

velocity.north = ekf_output_values(:, velocity_index.north);
velocity.east = ekf_output_values(:, velocity_index.east);
velocity.down = ekf_output_values(:, velocity_index.down);

orientation.roll = ekf_output_values(:, orientation_index.roll);
orientation.pitch = ekf_output_values(:, orientation_index.pitch);
orientation.yaw = ekf_output_values(:, orientation_index.yaw);

body_fixed_vel = zeros(length(time_from_start), 3);
body_fixed_vel_model = zeros(length(time_from_start), 4);

for i = 1:length(time_from_start)
	roll = orientation.roll(i);
	pitch = orientation.pitch(i);
	yaw = orientation.yaw(i);
	rot_matrix_from_n_to_b = ned_to_body_rotation_matrix(roll, pitch, yaw);
	ned_vel = [velocity.north(i); velocity.east(i); velocity.down(i)];
	body_fixed_vel(i, :) = (rot_matrix_from_n_to_b * ned_vel)';
end

for i = 1:length(time_from_start)
	non_linear_map_args.tau = [0; 0; total_vertical_thrust(i); 0];

	if i > 1
		integration_step_size = time_from_start(i) - time_from_start(i-1);
	else
		integration_step_size = 0.1;
	end

	body_fixed_vel_model(i+1, :) = rk4(body_fixed_vel_model(i, :)', body_fixed_vel_model(i, :)', ...
															 integration_step_size, @nonlinear_map, non_linear_map_args);
end

% Model uses the ENU frame, so the vertical velocity must be inverted
% TODO(mateusmenzes95): Fix this in the model
body_fixed_vel_model = -body_fixed_vel_model(2:end, :);

figure(1)
body_vel.w = body_fixed_vel(:, 3);
body_vel_model.w = body_fixed_vel_model(:, 3);

plot(time_from_start, total_vertical_thrust, 'k', 'LineWidth', 2)
ylabel('Total vertical thrust (N)')
xlabel('Time (s)')
yyaxis right
plot(time_from_start, body_vel.w, 'r', 'LineWidth', 2)
hold on
grid on
plot(time_from_start, body_vel_model.w, 'b', 'LineWidth', 2)
ylabel('Body-fixed vertical velocity (m/s)')
xlabel('Time (s)')
legend('Total vertical thrust', 'Body-fixed vertical velocity', 'Body-fixed vertical velocity (model)')

function thrust_array = pwm_to_thrust(pwm_array, direction)
	thrust_array = zeros(size(pwm_array));
	for i = 1:length(pwm_array)
		thrust_array(i) = pwm_to_thrust_single(pwm_array(i), direction);
	end
end

function thrust = pwm_to_thrust_single(pwm, direction)
% PWM_TO_THRUST_SINGLE Converts a single PWM value to thrust
%
%   thrust = PWM_TO_THRUST_SINGLE(pwm) takes a single PWM value (pwm) as input,
%   normalizes it, and then applies a polynomial function to calculate the corresponding
%   thrust. The polynomial function was derived from the BlueROV2's thruster data. See the script 
%   pwm_to_thrust_fit.py for more information on how the polynomial was derived.
%
%   Input:
%       pwm - A single raw PWM value extracted from the BlueROV2 datalog
%
%   Output:
%       thrust - The corresponding thrust value in Newtons
	x = pwm/1000;

	if direction == "cw"
		thrust = 17962.8799998694*(x^7) - 187769.011464511*(x^6) + 833815.144505776*(x^5) - 2038524.7018444*(x^4) ...
					 + 2963105.44270924*(x^3) - 2560904.9908962*(x^2) + 1218899.14844837*x - 246652.050873707;
		return
	end

	if direction == "ccw"
		thrust = -17962.8801069945*(x^7) + 189451.469642858*(x^6) - 848957.263024014*(x^5) + 2094807.52988121*(x^4) ...
						 -3073670.65232862*(x^3) + 2682026.83555928*(x^2) - 1289139.81322015*x + 263535.898637708;
		return
	end

	error("Invalid direction. Must be either 'cw' or 'ccw'");
end

function rot_matrix = ned_to_body_rotation_matrix(roll, pitch, yaw)
% NED_TO_BODY_ROTATION_MATRIX Returns the rotation matrix that transforms a vector from NED to body-fixed frame
%
%   rot_matrix = NED_TO_BODY_ROTATION_MATRIX(roll, pitch, yaw) takes the roll, pitch, and yaw angles as input
%   and returns the rotation matrix that transforms a vector from the NED frame to the body-fixed frame.
%
%   Input:
%       roll - The roll angle in degrees
%       pitch - The pitch angle in degrees
%       yaw - The yaw angle in degrees
%
%   Output:
%       rot_matrix - The 3x3 rotation matrix that performs the transformation

	phi = roll;
	theta = pitch;
	psi = yaw;

	c_psi = cosd(psi);
	s_psi = sind(psi);
	c_theta = cosd(theta);
	s_theta = sind(theta);
	c_phi = cosd(phi);
	s_phi = sind(phi);

	% The rotation matrix that transforms a vector from body-fixed frame to NED frame. Is is based of the equation
	% 2.18 in the book "Handbook of Marine Craft Hydrodynamics and Motion Control 2011" by Thor I. Fossen
	rot_matrix = ...
		[
			c_psi*c_theta, -s_psi*c_phi + c_psi*s_theta*s_phi, s_psi*s_phi + c_psi*c_phi*s_theta;
			s_psi*c_theta, c_psi*c_phi + s_phi*s_theta*s_psi, -c_psi*s_phi + s_theta*s_psi*c_phi;
			-s_theta, c_theta*s_phi, c_theta*c_phi
		];

	% Transpose the rotation matrix to convert from NED to body-fixed frame
	rot_matrix = rot_matrix';
end
