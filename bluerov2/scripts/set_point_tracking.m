clc
clear all

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

% Add paths so that we can use the functions from the files in the lib folder
addpath(genpath("."))
addpath(genpath("../../lib"))
addpath(genpath("../../lib/mpc_functions"))
addpath(genpath("../../lib/chart_functions/norms"))
addpath(genpath("../../lib/dynamic_models"))
addpath(genpath("../../lib/robot_models"))
addpath(genpath("../functions/matrices_getters"))

% Run some scripts to load the simulation parameters
run simulation_parameters
run bluerov2_models

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dynamic_model = nominal_model;
state_vector_size = size(dynamic_model.discrete_state_space.Ad, 1);
% Must change because the nominal model is positive buoyant
dynamic_model.gravity_vector = zeros(state_vector_size, 1);

integration_step = 0.001;
simulation_time = 100.0;
t = zeros(1, simulation_time/integration_step);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of simulation parameters section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% References parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set_point.begin = 5.0;
set_point.end = 70.0;

references = fill_references_array(state_vector_size, size(t, 2), set_point, integration_step, [0.2; 0.0; 0.0; 0.0]);
horizon_refs = zeros(prediction_horizon*state_vector_size, 1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Reference parameters section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MPC Tunning and initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Aaug = dynamic_model.augmented_state_space.Aaug;
Baug = dynamic_model.augmented_state_space.Baug;
Caug = dynamic_model.augmented_state_space.Caug;

q = 110;
r = 1000;

[Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
[kw, kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);

u = zeros(size(t, 2), 1);
u_last = zeros(state_vector_size, 1);

states_value = zeros(state_vector_size, simulation_time/integration_step);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of MPC Tunning and Initialization section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation loop. Integration performed with Runge-Kutta 4th order method
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
	u = delta_u(1:state_vector_size,1) + u_last;
	u_last = u;

	control_signal.X(k) = u(1);
	states_value(:, k+1) = rk4(@nonlinear_map, states_value(:, k), u, dynamic_model, integration_step);

	k = k + 1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of simulation loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Charts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
plot(t(1:end-1), control_signal.X)
grid on
xlabel('Time [s]')
ylabel('Control signal [N]')
title('Control signal X')

figure(2)
plot(t, states_value(1, :))
grid on
hold on
plot(t, references(1, :))
xlabel('Time [s]')
ylabel('Velocity [m]')
title('Velocity u')
labels = {'Velocity', 'Setpoint'};
legend(labels)
ylim([-0.05 0.15])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of charts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Functions used exclusively in this script
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xk_plus_1 = nonlinear_map(xk, tau, dynamic_model)
	M = dynamic_model.rigid_body_inertia_matrix + dynamic_model.added_mass_system_inertia_matrix;
	C_RB = get_rigid_body_coriolis_and_centripetal_matrix(xk, dynamic_model.mass);
	C_A = get_added_mass_coriolis_and_centripetal_matrix(xk, dynamic_model.added_mass_system_inertia_matrix);
	C = C_RB + C_A;
	D_v = get_quadratic_damping_matrix(xk, dynamic_model.quadratic_damping_coefficients);
	D = dynamic_model.linear_damping_matrix + D_v;
	G = dynamic_model.gravity_vector;
	xk_plus_1 = M\(tau - C*xk - D*xk);
end

function xk_plus_1 = rk4(f, xk, tau, dynamic_model, h)
  k1 = feval(f, xk, tau, dynamic_model);
  k2 = feval(f, xk + h/2*k1, tau, dynamic_model);
  k3 = feval(f, xk + h/2*k2, tau, dynamic_model);
  k4 = feval(f, xk + h*k3, tau, dynamic_model);
  xk_plus_1 = xk + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

function references = fill_references_array(num_states, num_samples, time_instants, dt, val_to_fill)
  references = zeros(num_states, num_samples);

  begin_index = time_instants.begin/dt;

	if begin_index == 0
		begin_index = 1;
	end

  end_index = time_instants.end/dt;

  k = begin_index;

  while k <= end_index
    references(:, k) = val_to_fill;
    k = k + 1;
  end
end
