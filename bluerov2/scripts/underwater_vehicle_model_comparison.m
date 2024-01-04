clc
clear all

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

% Add paths so that we can use the functions from the files in the lib folder
addpath(genpath("../lib"))
addpath(genpath("../lib/mpc_functions"))
addpath(genpath("../lib/chart_functions/norms"))
addpath(genpath("../lib/dynamic_models"))
addpath(genpath("../lib/robot_models"))
addpath(genpath("../bluerov2/functions"))

% Run some scripts to load the simulation parameters
run simulation_parameters
run bluerov2_model

models = [manzanilla_model; lipenitis_model; benzon_model; lower_model; upper_model; nominal_model];
linear_plot_line_styles = ["-r", '-b', '-g'];
nonlinear_plot_line_styles = ["--r", "--b", "--g"];

for j=1:size(models, 1)

dynamic_model = models(j);

state_vector_size = size(dynamic_model.discrete_state_space.Ad, 1);
states_name = {'u', 'v', 'w', 'r'};

dynamic_model.gravity_vector = zeros(state_vector_size, 1);

linear_x0 = zeros(state_vector_size, 1);
nonlinear_x0 = linear_x0;
tau = zeros(state_vector_size, 1);

integration_step = 0.001;
simulation_time = 100.0;

linear_x = zeros(state_vector_size, simulation_time/integration_step);
nonlinear_x = zeros(state_vector_size, simulation_time/integration_step);
position_x = zeros(state_vector_size, simulation_time/integration_step);
linear_x(:,1) = linear_x0;
nonlinear_x(:,1) = nonlinear_x0;
position_x(:,1) = nonlinear_x0;

t = zeros(1, simulation_time/integration_step);
tau = zeros(state_vector_size, simulation_time/integration_step);
k = 1;

max_vel = 0.2;

surge_step.begin = 5.0;
surge_step.end = 15.0;
surge_step.dt = integration_step;

sway_step.begin = 25.0;
sway_step.end = 50.0;
sway_step.dt = integration_step;

heave_step.begin = 60.0;
heave_step.end = 80.0;
heave_step.dt = integration_step;

yaw_step.begin = 90.0;
yaw_step.end = 95.0;
yaw_step.dt = integration_step;

generalized_force = [2.74; 0.0; 0.0; 0.0];
tau = fill_tau_vector(tau, surge_step, generalized_force);
generalized_force = [0.0; 1.5; 0.0; 0.0];
tau = fill_tau_vector(tau, sway_step, generalized_force);
generalized_force = [0.0; 0.0; 3.6; 0.0];
tau = fill_tau_vector(tau, heave_step, generalized_force);
generalized_force = [0.0; 0.0; 0.0; 1.0];
tau = fill_tau_vector(tau, yaw_step, generalized_force);

Ad = dynamic_model.discrete_state_space.Ad;
Bd = dynamic_model.discrete_state_space.Bd;
Cd = dynamic_model.discrete_state_space.Cd;

linear_x(:,k+1) = integration_step*(Ad*linear_x(:,k) + Bd*tau(:,k)) + linear_x(:,k);

start_time = cputime;

while true
  linear_x(:,k+1) = rk4(@linear_state_space_map, linear_x0, tau(:, k), dynamic_model, integration_step);
  nonlinear_x(:,k+1) = rk4(@nonlinear_map, nonlinear_x0, tau(:, k), dynamic_model, integration_step);

  linear_x0 = linear_x(:,k+1);
  nonlinear_x0 = nonlinear_x(:,k+1);

  k = k + 1;
  t(k) = k*integration_step;

  if t(end) >= simulation_time
      break
  end
end

time_elapsed = cputime - start_time;
print_section_description(['Simulation for model ' + dynamic_model.name + ' executed in ' + num2str(time_elapsed) + ' seconds'])

  if j == 1
    figure(1)
  end

  plot_robot_states(t, linear_x, nonlinear_x, 1.5, states_name, dynamic_model.name, dynamic_model.color)
end

function xk_plus_1 = linear_state_space_map(xk, tau, dynamic_model)
  system_inertia_matrix = dynamic_model.rigid_body_inertia_matrix + ...
    dynamic_model.added_mass_system_inertia_matrix;
  A = -system_inertia_matrix\dynamic_model.linear_damping_matrix;
  B = inv(system_inertia_matrix);
  xk_plus_1 = (A*xk + B*tau);
end

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

function xk_plus_1 = bypass_map(xk, tau, dynamic_model)
  xk_plus_1 = xk.*tau;
end

function xk_plus_1 = rk4(f, xk, tau, dynamic_model, h)
  k1 = feval(f, xk, tau, dynamic_model);
  k2 = feval(f, xk + h/2*k1, tau, dynamic_model);
  k3 = feval(f, xk + h/2*k2, tau, dynamic_model);
  k4 = feval(f, xk + h*k3, tau, dynamic_model);
  xk_plus_1 = xk + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

function v = get_velocity_vec(vel, dof)
  v = zeros(4, 1);
  if dof == "surge"
    v = [vel; 0.0; 0.0; 0.0];
    return
  end
  if dof == "sway"
    v = [0.0; vel; 0.0; 0.0];
    return
  end
  if dof == "heave"
    v = [0.0; 0.0; vel; 0.0];
    return
  end
  if dof == "yaw"
    v = [0.0; 0.0; 0.0; vel];
    return
  end
end

function vector = fill_tau_vector(tau_vector, time_instants, val_to_fill)
  vector = tau_vector;

  begin_index = time_instants.begin/time_instants.dt;
  end_index = time_instants.end/time_instants.dt;

  k = begin_index;

  while k <= end_index
    vector(:, k) = val_to_fill;
    k = k + 1;
  end
end

function plot_robot_states(t, linear_x, nonlinear_x, line_thickness, states_name, model_name, line_color)
  num_states = size(states_name, 2);

  for i=1:num_states
    subplot(num_states, 1, i)
    plot(t, linear_x(i, :), "-" + line_color, 'linewidth', line_thickness, 'DisplayName', model_name + " Linear")
    hold on
    plot(t, nonlinear_x(i, :), "--" + line_color, 'linewidth', line_thickness, 'DisplayName', model_name + " Nonlinear")
    if i == 1
      legend('show')
    end
    grid on
    xlabel('Tempo (s)');
    if i < num_states
    ylim([0 0.25])
    ylabel([states_name{i} ' [m/s]']);
    else
      ylabel([states_name{i} ' [rad/s]']);
    end
  end
end
