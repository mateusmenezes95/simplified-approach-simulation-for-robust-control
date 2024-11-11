clc
clear

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

% Add paths so that we can use the functions from the files in the lib folder
addpath(genpath("."))
addpath(genpath("../functions/matrices_getters"))
addpath(genpath("../functions/numerical_integration"))
addpath(genpath("../../lib/mpc_functions"))
addpath(genpath("../../lib/utils"))

% Run some scripts to load the simulation parameters
run bluerov2_simulation_parameters
run bluerov2_models
run trajectory_generation

%===================================================================================================
% Simulation parameters
%===================================================================================================
dynamic_model = nominal_model;
state_vector_size = size(dynamic_model.discrete_state_space.Ad, 1);

vehicle_weight = dynamic_model.mass*gravity_constant;
vehicle_buoyancy = water_density*gravity_constant*dynamic_model.volume;
z_restoring_force = -(vehicle_weight - vehicle_buoyancy);
dynamic_model.gravity_vector = [0; 0; z_restoring_force; 0];

integration_step_ratio = 50;
integration_step_size = sampling_period/integration_step_ratio;

simulation_time = total_time;
end_time = ceil(simulation_time/sampling_period)*sampling_period;

time = 0:integration_step_size:simulation_time;
num_of_simulation_steps = length(time);
sim_time = zeros(1, num_of_simulation_steps);

num_of_samples = ceil(simulation_time/sampling_period);
samples_delayed = 14;
%===================================================================================================
% End of simulation parameters section
%===================================================================================================

%===================================================================================================
% Plot Parameters
%===================================================================================================
font_size = 10;
line_thickness = 1.5;
y_axis_limits_offset = 0.2;
figure_idx = 1;
save_graph_flag = false;
%==============================================================================

%===================================================================================================
% MPC Tunning and initialization
%===================================================================================================
Aaug = dynamic_model.augmented_state_space.Aaug;
Baug = dynamic_model.augmented_state_space.Baug;
Caug = dynamic_model.augmented_state_space.Caug;

surge.q = 7500;
surge.r = 200;

sway.q = 7500;
sway.r = 200;

heave.q = 7500;
heave.r = 200;

yaw.q = 1500;
yaw.r = 200;

q = diag([surge.q, sway.q, heave.q, yaw.q]);
r = diag([surge.r, sway.r, heave.r, yaw.r]);

[Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
[kw, kmpc, Q, R] = get_mpc_gains_non_scalar_qr(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);
kmpc = kmpc(1:state_vector_size,:);
kw = kw(1:state_vector_size, :);

params.state_vector_size = state_vector_size;
params.prediction_horizon = prediction_horizon;
params.current_time_step = 1;
params.navigation_velocity = 0.1;

waypoints = generate_square_trajectory(1, params.navigation_velocity, sampling_period);
vel_ref = desired.body_fixed_vel;
%===================================================================================================
% End of MPC Tunning and Initialization section
%===================================================================================================

%===================================================================================================
% Initial conditions section
%===================================================================================================
body_fixed_vel = zeros(state_vector_size, num_of_simulation_steps+1);  % v(:, 1) = [0; 0; 0; 0] -> Initial condition
body_fixed_vel_sampled = zeros(state_vector_size, num_of_samples+1);

position_and_attitude = zeros(state_vector_size, num_of_simulation_steps+1);
position_and_attitude(:, 1) = [0; 0; 5; 0];
position_and_attitude_sampled = zeros(state_vector_size, num_of_samples+1);
position_and_attitude_sampled(:, 1) = position_and_attitude(:, 1);

ned_velocities = zeros(state_vector_size, num_of_simulation_steps+1);
ned_velocities_sampled = zeros(num_of_samples+1, state_vector_size);

generalized_forces = zeros(state_vector_size, num_of_simulation_steps+1);
generalized_forces_sampled = zeros(state_vector_size, num_of_samples+1);

non_linear_map_args.dynamic_model = dynamic_model;
position_and_attitude_args.roll = 0;
position_and_attitude_args.pitch = 0;

delta_xk = zeros(state_vector_size, 1);
control_signal = zeros(state_vector_size, 1);

k = 1;
%===================================================================================================
% End of Initial conditions section
%===================================================================================================

%===================================================================================================
% Simulation loop. Integration performed with Runge-Kutta 4th order method
%===================================================================================================
for i=1:num_of_simulation_steps
	sim_time(i) = (i-1)*integration_step_size;

	% Sample instant
	if (mod(i, integration_step_ratio) == 1 || i == 1)
		horizon_refs = generate_horizon_vel_ref(vel_ref, params);
		horizon_ref(1:state_vector_size, k) = horizon_refs(1:state_vector_size);
		params.current_time_step = params.current_time_step + 1;

		body_fixed_vel_sampled(:, k) = body_fixed_vel(:, i);

		if (i > 1)
			delta_xk = body_fixed_vel_sampled(:, k) - body_fixed_vel_sampled(:, k-1);
		end

		ksi = [delta_xk; body_fixed_vel_sampled(:, k)];    % In this case, y[k] is the state vector due to C = I
		delta_u = kw*horizon_refs - kmpc*ksi;
	
		if (k > 1)
			control_signal(:, k) = delta_u(1:state_vector_size,1) + control_signal(:, k-1);
		else
			control_signal(:, k) = delta_u(1:state_vector_size,1);
		end

		position_and_attitude_sampled(:, k) = position_and_attitude(:, i);
		ned_velocities_sampled(k, :) = ned_velocities(:, i);
		generalized_forces_sampled(:, k) = generalized_forces(:, i);

		k = k + 1;
	end

	if (k > samples_delayed + 1)
		generalized_forces(:, i) = control_signal(:, k - samples_delayed - 1);
	else 
		generalized_forces(:, i) = control_signal(:, 1);
	end

	non_linear_map_args.tau = generalized_forces(:, i);

	body_fixed_vel(:, i+1) = rk4(body_fixed_vel(:, i), body_fixed_vel(:, i), ...
															 integration_step_size, @nonlinear_map, non_linear_map_args);

	position_and_attitude_args.yaw = position_and_attitude(4, i);

	ned_velocities(:, i+1) = body_fixed_to_inertial_frame(body_fixed_vel(:, i+1), position_and_attitude_args);
	position_and_attitude(:, i+1) = rk4(position_and_attitude(:, i), body_fixed_vel(:, i+1), ...
																			integration_step_size, @body_fixed_to_inertial_frame, ...
																			position_and_attitude_args);
end

% body_fixed_vel(:, 1) is the initial condition, so we remove it
body_fixed_vel = body_fixed_vel(:, 2:end);
body_fixed_vel_sampled = body_fixed_vel_sampled(:, 2:end);

position_and_attitude = position_and_attitude(:, 2:end);
position_and_attitude_sampled = position_and_attitude_sampled(:, 2:end);

ned_velocities = ned_velocities(:, 2:end);
ned_velocities_sampled = ned_velocities_sampled(2:end, :);

generalized_forces = generalized_forces(:, 2:end);
generalized_forces_sampled = generalized_forces_sampled(:, 2:end);

rmse = calculate_rmse(desired.trajectory, position_and_attitude_sampled');

%===================================================================================================
% End of simulation loop
%===================================================================================================

%===================================================================================================
% Charts
%===================================================================================================
fig_name_suffix = "-" + num2str(samples_delayed) + "-samples-delayed";
figure("Name", "desired-versus-actual-trajectory-with" + fig_name_suffix)
desired_trajectory_args.y_labels = {'x [m]', 'y [m]', 'z [m]', '$\psi$ [rad]'};
desired_trajectory_args.y_min_offset = 0.1;
desired_trajectory_args.y_max_offset = 0.1;
plot_per_dof_values(t, desired_trajectory_args, desired.trajectory, position_and_attitude_sampled')
save_graph(save_graph_flag, base_path_for_fig_save)

figure("Name", "velocities-in-body-fixed-frame" + fig_name_suffix)
desired_body_fixed_vel_args.y_labels = {'u [m/s]', 'v [m/s]', 'w [m/s]', 'r [rad/s]'};
desired_body_fixed_vel_args.y_min_offset = 0.1;
desired_body_fixed_vel_args.y_max_offset = 0.1;
plot_per_dof_values(t, desired_body_fixed_vel_args, desired.body_fixed_vel, body_fixed_vel_sampled')
save_graph(save_graph_flag, base_path_for_fig_save)

figure("Name", "bluerov-control-signals" + fig_name_suffix)
bluerov_control_signals_args.y_labels = {'$X$ [N]', '$Y$ [N]', '$Z$ [N]', '$N$ [Nm]'};
bluerov_control_signals_args.y_min_offset = 1.0;
bluerov_control_signals_args.y_max_offset = 1.0;
plot_per_dof_values(t, bluerov_control_signals_args, control_signal(:, 2:end)')
save_graph(save_graph_flag, base_path_for_fig_save)

actual.pose = position_and_attitude';
actual.line_spec = '-b';
actual.line_width = line_thickness;

desired.pose = desired.trajectory;
desired.line_spec = '--r';
desired.line_width = 1.5;

figure("Name", "bluerov-3d-trajectory" + fig_name_suffix)
plot_3d_path(desired, actual)
save_graph(save_graph_flag, base_path_for_fig_save)
% %===================================================================================================
% End of charts
%===================================================================================================

%===================================================================================================
% Functions used exclusively in this script
%===================================================================================================
function ned_vel = body_fixed_to_inertial_frame(body_fixed_vel, arg)
	roll = arg.roll;
	pitch = arg.pitch;
	yaw = arg.yaw;
	body_fixed_to_ned_rot = [
		cos(yaw) -sin(yaw) 0 0; ...
		sin(yaw)  cos(yaw) 0 0; ...
		   0         0     1 0; ...
		   0         0     0 1];
	ned_vel = body_fixed_to_ned_rot*body_fixed_vel;
end

function horizon_vel_ref = generate_horizon_vel_ref(vel_trajectory, const_param)
	state_vector_size = const_param.state_vector_size;
	k = const_param.current_time_step;
	Np = const_param.prediction_horizon;

	horizon_vel_ref = zeros(Np*state_vector_size,1);

	i=1;
	for j=k:k+Np-1
			if j > length(vel_trajectory)
				temp_vec = [vel_trajectory(end, 1); vel_trajectory(end, 2); vel_trajectory(end, 3); vel_trajectory(end, 4)];
			else
				temp_vec = [vel_trajectory(j,1); vel_trajectory(j,2); vel_trajectory(j,3); vel_trajectory(j,4)];
			end
			horizon_vel_ref(i:i+state_vector_size-1,1) = temp_vec;
			i=i+state_vector_size;
	end
end

function waypoints = generate_square_trajectory(square_size, nav_vel, sampling_period)
  path_nav_time = (square_size*4)/nav_vel;
  waypoints_qty = ceil((path_nav_time/sampling_period)/4);

  x1 = linspace(0, 1, waypoints_qty);
  theta = zeros(1, length(x1))';
  y1 = linspace(0, 1, waypoints_qty);
  theta = [theta; deg2rad(ones(1,length(y1))*90)'];
  %In the 1x1m square, instead of starting from 1 again, shift to the next
  %element, in this case 0.9970
  x2 = linspace(x1(end-1), 0, waypoints_qty);
  theta = [theta; deg2rad(ones(1,length(x2))*180)'];
  y2 = linspace(y1(end-1),0, waypoints_qty);
  theta = [theta; deg2rad(ones(1,length(y2))*270)'];

  x = [x1'; ones(1,length(y1))'; x2'; zeros(1, length(y2))'];
  y = [zeros(1, length(x1))'; y1'; ones(1, length(x2))'; y2'];
	z = zeros(1, length(x))';

  waypoints = [x y z theta]';
end

function plot_generalized_forces (t, u, legend_name, line_spec, line_thickness, ylabel_prefix)
	generalized_forces_name = {'X', 'Y', 'Z', 'N'}; % According to SNAME notation
	control_signals = size(generalized_forces_name, 2);
	limit_offset = 0.5;

	for i=1:control_signals
		subplot(control_signals, 1, i)

		if(legend_name ~= -1)
			plot(t, u(i,:), line_spec, 'linewidth', line_thickness, 'DisplayName', legend_name)
			legend(Location="best")
		else
			plot(t, u(i,:), line_spec, 'linewidth', line_thickness)
		end
		hold on
		grid on
		xlabel('Time [s]');

    if i < control_signals
      ylabel([generalized_forces_name{i} ' [N]']);
    else
      ylabel([generalized_forces_name{i} ' [Nm]']);
    end

		ylim([(min(u(i,:)) - limit_offset) (max(u(i,:)) + limit_offset)])

		if i == 1
			title('Generalized forces (control signals)')
		end
	end
end

function rmse = calculate_rmse(desired, actual)
	rmse = zeros(1, size(desired, 2));
	for i=1:size(desired, 2)
		sum = 0;
		for j=1:size(desired, 1)
			sum = sum + (desired(j, i) - actual(j, i))^2;
		end
		rmse(i) = sqrt(sum/size(desired, 1));
	end
end
