clc
clear all

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

% Add paths so that we can use the functions from the files in the lib folder
addpath(genpath("."))
addpath(genpath("../functions/matrices_getters"))
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

integration_step = 0.001;
simulation_time = 100.0;
num_of_simulation_steps = simulation_time/integration_step;
t = zeros(1, simulation_time/integration_step);
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
% Body-fixed velocity references parameters
%===================================================================================================
path_step_size = 0.01;
path.x = 0:path_step_size:1;
path.x = [path.x ones(1, 99)];
temp = 0.01:path_step_size:(1-path_step_size);
path.y = [zeros(1,101) temp];
% references = fill_references_array(vel_references_struct);
% horizon_refs = zeros(prediction_horizon*state_vector_size, 1);
%===================================================================================================
% End of Reference parameters section
%===================================================================================================

%===================================================================================================
% MPC Tunning and initialization
%===================================================================================================
Aaug = dynamic_model.augmented_state_space.Aaug;
Baug = dynamic_model.augmented_state_space.Baug;
Caug = dynamic_model.augmented_state_space.Caug;

q = 100;
r = 1000;

[Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
[kw, kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);

u = zeros(size(t, 2), 1);
u_last = zeros(state_vector_size, 1);

states_value = zeros(state_vector_size, num_of_simulation_steps);
generalized_forces = zeros(state_vector_size, num_of_simulation_steps);

params.state_vector_size = state_vector_size;
params.prediction_horizon = prediction_horizon;
params.current_time_step = 1;
params.navigation_velocity = 0.2;

waypoints = generate_square_trajectory(1, params.navigation_velocity, integration_step);
horizon_refs = get_vel_horizon_refs(states_value(:, 1), waypoints, params);
%===================================================================================================
% End of MPC Tunning and Initialization section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%===================================================================================================
% Simulation loop. Integration performed with Runge-Kutta 4th order method
%===================================================================================================
k = 1;
for h=0:integration_step:simulation_time
	t(k) = (k-1)*integration_step;
	if t(end) >= simulation_time - integration_step
		break
	end

	if k == 1
		delta_xk = zeros(state_vector_size, 1);
	else
		delta_xk = states_value(:, k) - states_value(:, k-1);
	end

	current_pose = rk4(@body_fixed_to_inertial_frame, states_value(:, k), generalized_forces(:, k), dynamic_model, integration_step);
	horizon_refs = get_vel_horizon_refs(current_pose, waypoints, params);

	ksi = [delta_xk; states_value(:, k)];    % In this case, y[k] is the state vector due to C = I
	delta_u = kw*horizon_refs - kmpc*ksi;

	if k == 1
		generalized_forces(:, k) = delta_u(1:state_vector_size,1);
	else
		generalized_forces(:, k) = delta_u(1:state_vector_size,1) + generalized_forces(:, k-1);
	end

	states_value(:, k+1) = rk4(@nonlinear_map, states_value(:, k), generalized_forces(:, k), dynamic_model, integration_step);

	if rem(h, sampling_period) == 0
		k = k + 1;
	end
end
%===================================================================================================
% End of simulation loop
%===================================================================================================

%===================================================================================================
% Charts
%===================================================================================================
figure("Name", "bluerov-states")
plot_bluerov_states(t, states_value, '-r', line_thickness)

figure("Name", "bluerov-control-signals")
plot_generalized_forces(t, generalized_forces, -1, '-r', line_thickness)
% %===================================================================================================
% End of charts
%===================================================================================================

%===================================================================================================
% Functions used exclusively in this script
%===================================================================================================
function x = nonlinear_map(xk, tau, dynamic_model)
	M = dynamic_model.rigid_body_inertia_matrix + dynamic_model.added_mass_system_inertia_matrix;
	C_RB = get_rigid_body_coriolis_and_centripetal_matrix(xk, dynamic_model.mass);
	C_A = get_added_mass_coriolis_and_centripetal_matrix(xk, dynamic_model.added_mass_system_inertia_matrix);
	C = C_RB + C_A;
	D_v = get_quadratic_damping_matrix(xk, dynamic_model.quadratic_damping_coefficients);
	D = dynamic_model.linear_damping_matrix + D_v;
	G = dynamic_model.gravity_vector;
	x = M\(tau - C*xk - D*xk - G);
end

function eta = body_fixed_to_inertial_frame(xk, tau, dynamic_model)
	R = [cos(xk(4)) -sin(xk(4)) 0 0; ...
			 sin(xk(4)) cos(xk(4)) 0 0; ...
			 0 0 1 0; ...
			 0 0 0 1];
	eta = R*xk;
end

function xk_plus_1 = rk4(f, xk, tau, dynamic_model, h)
  k1 = feval(f, xk, tau, dynamic_model);
  k2 = feval(f, xk + h/2*k1, tau, dynamic_model);
  k3 = feval(f, xk + h/2*k2, tau, dynamic_model);
  k4 = feval(f, xk + h*k3, tau, dynamic_model);
  xk_plus_1 = xk + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

function references = fill_references_array(vel_ref_struct_arg)
  references = zeros(vel_ref_struct_arg.num_states, vel_ref_struct_arg.num_samples);
	dt = vel_ref_struct_arg.integration_step;

  vel_ref_struct_arg.u.begin_index = vel_ref_struct_arg.u.begin_time/dt;
	vel_ref_struct_arg.u.end_index = vel_ref_struct_arg.u.end_time/dt;
	references(1, vel_ref_struct_arg.u.begin_index:vel_ref_struct_arg.u.end_index) = vel_ref_struct_arg.u.value;

	vel_ref_struct_arg.v.begin_index = vel_ref_struct_arg.v.begin_time/dt;
	vel_ref_struct_arg.v.end_index = vel_ref_struct_arg.v.end_time/dt;
	references(2, vel_ref_struct_arg.v.begin_index:vel_ref_struct_arg.v.end_index) = vel_ref_struct_arg.v.value;

	vel_ref_struct_arg.w.begin_index = vel_ref_struct_arg.w.begin_time/dt;
	vel_ref_struct_arg.w.end_index = vel_ref_struct_arg.w.end_time/dt;
	references(3, vel_ref_struct_arg.w.begin_index:vel_ref_struct_arg.w.end_index) = vel_ref_struct_arg.w.value;

	vel_ref_struct_arg.r.begin_index = vel_ref_struct_arg.r.begin_time/dt;
	vel_ref_struct_arg.r.end_index = vel_ref_struct_arg.r.end_time/dt;
	references(4, vel_ref_struct_arg.r.begin_index:vel_ref_struct_arg.r.end_index) = vel_ref_struct_arg.r.value;
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

	if (k > length(waypoints))
			return
	end

	i=1;
	for j=k:k+Np-1
			Rz = [cos(waypoints(j)) sin(waypoints(j)) 0 0; ...
					 -sin(waypoints(j)) cos(waypoints(j)) 0 0; ...
					 			0 												0 		1	0; ...
								0													0			0	1];

			if j > length(waypoints)
					x_ref = waypoints(1,end);
					y_ref = waypoints(2,end);
					z_ref = waypoints(3,end);
					psi_ref = waypoints(4,end);
			else
					x_ref = waypoints(1,j);
					y_ref = waypoints(2,j);
					z_ref = waypoints(3,j);
					psi_ref = waypoints(4,j);
			end

			beta = atan2(y_ref-y, x_ref-x);  % atan2(yr(k + j|k) - yr(k), xr(k + j|k) − xr(k))
			temp_vec = [nav_vel*cos(beta) nav_vel*sin(beta) z_ref-z psi_ref-psi]';
			temp_vec = Rz*temp_vec;
			horizon_refs(i:i+state_vector_size-1,1) = temp_vec;
			i=i+state_vector_size;
	end
end

function waypoints = generate_square_trajectory(square_size, nav_vel, sampling_period)
  path_nav_time = (square_size*4)/nav_vel;
  waypoints_qty = ceil((path_nav_time/sampling_period)/4);

  x1 = linspace(0,1, waypoints_qty);
  theta = zeros(1,length(x1))';
  y1 = linspace(0,1, waypoints_qty);
  theta = [theta; deg2rad(ones(1,length(y1))*90)'];
  %In the 1x1m square, instead of starting from 1 again, shift to the next
  %element, in this case 0.9970
  x2 = linspace(x1(end-1),0, waypoints_qty);
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
