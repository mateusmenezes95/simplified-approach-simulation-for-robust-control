clc
clear all

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

% Add paths so that we can use the functions from the files in the lib folder
addpath(genpath("."))
addpath(genpath("../functions/"))
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
vel_references_struct.num_states = state_vector_size;
vel_references_struct.num_samples = size(t, 2);
vel_references_struct.integration_step = integration_step;

vel_references_struct.u.begin_time = 5.0;	% s
vel_references_struct.u.end_time = 30.0;		% s
vel_references_struct.u.value = 0.1;  % m/s

vel_references_struct.v.begin_time = 25.0;	% s
vel_references_struct.v.end_time = 50.0;		% s
vel_references_struct.v.value = 0.1;	% m/s

vel_references_struct.w.begin_time = 60.0;	% s
vel_references_struct.w.end_time = 90.0;		% s
vel_references_struct.w.value = 0.1;	% m/s

vel_references_struct.r.begin_time = 90.0;
vel_references_struct.r.end_time = 95.0;
vel_references_struct.r.value = (pi/2)/10;	% rad/s ((pi/2)/10 = 10 deg/s)

references = fill_references_array(vel_references_struct);
horizon_refs = zeros(prediction_horizon*state_vector_size, 1);
%===================================================================================================
% End of Reference parameters section
%===================================================================================================

%===================================================================================================
% MPC Tunning and initialization
%===================================================================================================
Aaug = dynamic_model.augmented_state_space.Aaug;
Baug = dynamic_model.augmented_state_space.Baug;
Caug = dynamic_model.augmented_state_space.Caug;

q = 110;
r = 1000;

[Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
[kw, kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);

u = zeros(size(t, 2), 1);
u_last = zeros(state_vector_size, 1);

states_value = zeros(state_vector_size, num_of_simulation_steps);
generalized_forces = zeros(state_vector_size, num_of_simulation_steps);
%===================================================================================================
% End of MPC Tunning and Initialization section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%===================================================================================================
% Simulation loop. Integration performed with Runge-Kutta 4th order method
%===================================================================================================
k = 1;
while true
	t(k) = (k-1)*integration_step;
	if t(end) >= simulation_time - integration_step
		break
	end

	if k == 1
		delta_xk = zeros(state_vector_size, 1);
	else
		delta_xk = states_value(:, k) - states_value(:, k-1);
	end

	i = 1;
	for j=k:k+prediction_horizon-1
		if j > size(references, 2)
			horizon_refs(i:i+state_vector_size-1, 1) = references(:, end);
		else
			horizon_refs(i:i+state_vector_size-1, 1) = references(:, j);
			vel(j) = references(1, j);
		end
		i = i + state_vector_size;
	end

	ksi = [delta_xk; states_value(:, k)];    % In this case, y[k] is the state vector due to C = I
	delta_u = kw*horizon_refs - kmpc*ksi;

	if k == 1
		generalized_forces(:, k) = delta_u(1:state_vector_size,1);
	else
		generalized_forces(:, k) = delta_u(1:state_vector_size,1) + generalized_forces(:, k-1);
	end

	states_value(:, k+1) = rk4(@nonlinear_map, states_value(:, k), generalized_forces(:, k), dynamic_model, integration_step);

	k = k + 1;
end
%===================================================================================================
% End of simulation loop
%===================================================================================================

%===================================================================================================
% Charts
%===================================================================================================
figure("Name", "bluerov-states")
plot_bluerov_states(t, states_value, references, '-r', line_thickness)

figure("Name", "bluerov-control-signals")
plot_generalized_forces(t, generalized_forces, -1, '-r', line_thickness)
%===================================================================================================
% End of charts
%===================================================================================================

%===================================================================================================
% Functions used exclusively in this script
%===================================================================================================
function xk_plus_1 = nonlinear_map(xk, tau, dynamic_model)
	M = dynamic_model.rigid_body_inertia_matrix + dynamic_model.added_mass_system_inertia_matrix;
	C_RB = get_rigid_body_coriolis_and_centripetal_matrix(xk, dynamic_model.mass);
	C_A = get_added_mass_coriolis_and_centripetal_matrix(xk, dynamic_model.added_mass_system_inertia_matrix);
	C = C_RB + C_A;
	D_v = get_quadratic_damping_matrix(xk, dynamic_model.quadratic_damping_coefficients);
	D = dynamic_model.linear_damping_matrix + D_v;
	G = dynamic_model.gravity_vector;
	xk_plus_1 = M\(tau - C*xk - D*xk - G);
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
