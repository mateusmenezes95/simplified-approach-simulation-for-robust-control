clc
clear all

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

%===================================================================================================
% Simulation parameters
%===================================================================================================
dynamic_model = nominal_model;
state_vector_size = size(dynamic_model.discrete_state_space.Ad, 1);
dynamic_model.gravity_vector = [0; 0; 2.5; 0];

integration_step_ratio = 50;
integration_step_size = sampling_period/integration_step_ratio;

simulation_time = 50;
end_time = ceil(simulation_time/sampling_period)*sampling_period;

time = 0:integration_step_size:simulation_time;
num_of_simulation_steps = length(time);
sim_time = zeros(1, num_of_simulation_steps);

num_of_samples = ceil(simulation_time/sampling_period);
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
%==============================================================================

%===================================================================================================
% MPC Tunning and initialization
%===================================================================================================
Aaug = dynamic_model.augmented_state_space.Aaug;
Baug = dynamic_model.augmented_state_space.Baug;
Caug = dynamic_model.augmented_state_space.Caug;

q = 9500;
r = 200;

[Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
[kw, kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);

params.state_vector_size = state_vector_size;
params.prediction_horizon = prediction_horizon;
params.current_time_step = 1;
params.navigation_velocity = 0.1;

waypoints = generate_square_trajectory(1, params.navigation_velocity, sampling_period);
%===================================================================================================
% End of MPC Tunning and Initialization section
%===================================================================================================

%===================================================================================================
% Initial conditions section
%===================================================================================================
body_fixed_vel = zeros(state_vector_size, num_of_simulation_steps+1);  % v(:, 1) = [0; 0; 0; 0] -> Initial condition
body_fixed_vel_sampled = zeros(state_vector_size, num_of_samples+1);

position_and_attitude = zeros(state_vector_size, num_of_simulation_steps+1);
position_and_attitude_sampled = zeros(state_vector_size, num_of_samples+1);

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
		horizon_refs = get_vel_horizon_refs(position_and_attitude(:, i), waypoints, params);
		horizon_ref(1:state_vector_size, k) = horizon_refs(1:state_vector_size);
		params.current_time_step = params.current_time_step + 1;

		body_fixed_vel_sampled(:, k) = body_fixed_vel(:, i);

		if (i > 1)
			delta_xk = body_fixed_vel_sampled(:, k) - body_fixed_vel_sampled(:, k-1);
		end

		ksi = [delta_xk; body_fixed_vel_sampled(:, k)];    % In this case, y[k] is the state vector due to C = I
		delta_u = kw*horizon_refs - kmpc*ksi;
	
		control_signal = delta_u(1:state_vector_size,1) + control_signal;
		k = k + 1;
	end

	generalized_forces(:, i) = control_signal;
	non_linear_map_args.tau = control_signal;

	body_fixed_vel(:, i+1) = rk4(body_fixed_vel(:, i), body_fixed_vel(:, i), ...
															 integration_step_size, @nonlinear_map, non_linear_map_args);

	position_and_attitude_args.yaw = position_and_attitude(4, i);

	position_and_attitude(:, i+1) = rk4(position_and_attitude(:, i), body_fixed_vel(:, i+1), ...
																			integration_step_size, @body_fixed_to_inertial_frame, ...
																			position_and_attitude_args);
end

% body_fixed_vel(:, 1) is the initial condition, so we remove it
body_fixed_vel = body_fixed_vel(:, 2:end);
position_and_attitude = position_and_attitude(:, 2:end);
generalized_forces = generalized_forces(:, 2:end);

%===================================================================================================
% End of simulation loop
%===================================================================================================

%===================================================================================================
% Charts
%===================================================================================================
figure("Name", "bluerov-states")
plot_bluerov_states(sim_time, body_fixed_vel, '-r', line_thickness)

figure("Name", "bluerov-control-signals")
plot_generalized_forces(sim_time, generalized_forces, -1, '-r', line_thickness)

figure("Name", "bluerov-3d-trajectory")
plot3(position_and_attitude(1,:), ...
			position_and_attitude(2,:), ...
			position_and_attitude(3,:), ...
			'-r', 'linewidth', line_thickness)
grid on

figure("Name", "bluerov-2d-trajectory")
plot(position_and_attitude(1,:), ...
			position_and_attitude(2,:), ...
			'-r', 'linewidth', line_thickness)
grid on
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
		sin(yaw) cos(yaw) 0 0; ...
		0 0 1 0; ...
		0 0 0 1];
	ned_vel = body_fixed_to_ned_rot*body_fixed_vel;
end

function horizon_refs = get_vel_horizon_refs(current_pose, waypoints, const_param)
	% Future references must be in Y = [yr(k) yr(k+1) ... yr(k+N-1)]
	state_vector_size = const_param.state_vector_size;
	nav_vel = const_param.navigation_velocity;
	k = const_param.current_time_step;
	Np = const_param.prediction_horizon;

	horizon_refs = zeros(Np*state_vector_size,1);
	x=current_pose(1);
	y=current_pose(2);
	z=current_pose(3);
	psi=current_pose(4);

	i=1;
	for j=k:k+Np-1
			if j > length(waypoints)
					x_ref = waypoints(1,end);
					y_ref = waypoints(2,end);
					z_ref = waypoints(3,end);
					psi_ref = waypoints(4,end);
					beta = deg2rad(-90);
			else
					x_ref = waypoints(1,j);
					y_ref = waypoints(2,j);
					z_ref = waypoints(3,j);
					psi_ref = waypoints(4,j);
					beta = atan2(y_ref-y, x_ref-x);  % atan2(yr(k + j|k) - yr(k), xr(k + j|k) − xr(k))
			end

			Rz = [cos(psi_ref) sin(psi_ref) 0 0; ...
					 -sin(psi_ref) cos(psi_ref) 0 0; ...
					 	     0 					 0 				1	0; ...
						     0					 0				0	1];

			temp_vec = [nav_vel*cos(beta) nav_vel*sin(beta) 0 (psi_ref-psi)]';
			temp_vec = Rz*temp_vec;
			horizon_refs(i:i+state_vector_size-1,1) = temp_vec;
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

function plot_bluerov_states(t, x, line_spec, line_thickness)
  states_name = {'u', 'v', 'w', 'r'};
  num_states = size(states_name, 2);
	limit_offset = 0.05;

  for i=1:num_states
    subplot(num_states, 1, i)

    hold on
    plot(t, x(i,:), line_spec, 'linewidth', line_thickness, 'DisplayName', ['velocity ' states_name{i}])
    hold off
    legend('show', 'location', 'northeast')

    grid on
    xlabel('Time [s]');
    if i < num_states
      ylabel([states_name{i} ' [m/s]']);
    else
      ylabel([states_name{i} ' [rad/s]']);
    end

    min_value = min(x(i,:));
    max_value = max(x(i,:));

		ylim([(min_value - limit_offset) (max_value + limit_offset)])

		if i == 1
			title('Body-fixed velocities (robot states)')
		end

  end
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
