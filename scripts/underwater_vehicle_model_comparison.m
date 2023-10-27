clc
clear

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

addpath(genpath("../lib"))
addpath(genpath("../lib/mpc_functions"))
addpath(genpath("../lib/chart_functions/norms"))
addpath(genpath("../lib/dynamic_models"))
addpath(genpath("../lib/robot_models"))

run simulation_parameters
run bluerov2_model

dynamic_model.mass = mass;
dynamic_model.quadratic_damping_coefficient = quadratic_damping_coefficient;
dynamic_model.rigid_body_inertia_matrix = rigit_body_inertia_matrix;
dynamic_model.added_mass_inertia_matrix = added_mass_inertia_matrix;
dynamic_model.linear_damping_matrix = linear_damping_matrix;
dynamic_model.quadratic_damping_matrix = zeros(state_vector_size, state_vector_size);
dynamic_model.gravity_vector = zeros(state_vector_size, 1);

linear_x0 = zeros(state_vector_size, 1);
nonlinear_x0 = linear_x0;
tau = zeros(state_vector_size, 1);
tau(1) = 8.5;

integration_step = 0.001;
simulation_time = 25.0;

linear_x = zeros(state_vector_size, simulation_time/integration_step);
nonlinear_x = zeros(state_vector_size, simulation_time/integration_step);
linear_x(:,1) = linear_x0;
nonlinear_x(:,1) = nonlinear_x0;
t = zeros(1, simulation_time/integration_step);
k = 1;

linear_x(:,k+1) = integration_step*(A*linear_x(:,k) + B*tau) + linear_x(:,k);

while true
  % if t(k) >= 5.0
  %   tau(2) = 10.0;
  % end
  % if t(k) >= 15.0
  %   tau(3) = 10.0;
  % end
  tau(4) = 0.1*sin(2*pi*0.1*integration_step*k);
  linear_x(:,k+1) = rk4(@linear_state_space_map, linear_x0, tau, dynamic_model, integration_step);
  nonlinear_x(:,k+1) = rk4(@nonlinear_map, nonlinear_x0, tau, dynamic_model, integration_step);
  linear_x0 = linear_x(:,k+1);
  nonlinear_x0 = nonlinear_x(:,k+1);
  k = k + 1;
  t(k) = k*integration_step;
  if t(end) >= simulation_time
      break
  end
end

plot_robot_states(t, linear_x, nonlinear_x, 1, states)

function xk_plus_1 = linear_state_space_map(xk, tau, dynamic_model)
  system_inertia_matrix = dynamic_model.rigid_body_inertia_matrix + ...
    dynamic_model.added_mass_inertia_matrix;
  A = -system_inertia_matrix\dynamic_model.linear_damping_matrix;
  B = inv(system_inertia_matrix);
  xk_plus_1 = (A*xk + B*tau);
end

function xk_plus_1 = nonlinear_map(xk, tau, dynamic_model)
  M = dynamic_model.rigid_body_inertia_matrix + dynamic_model.added_mass_inertia_matrix;
  C_RB = coriolis_matrix_rb(xk, dynamic_model.mass);
  C_A = coriolis_matrix_a(xk, dynamic_model.added_mass_inertia_matrix);
  C = C_RB + C_A;
  D_v = quadratic_damping_matrix(xk, dynamic_model.quadratic_damping_coefficient);
  D = dynamic_model.linear_damping_matrix + D_v;
  G = dynamic_model.gravity_vector;
  xk_plus_1 = M\(tau - C*xk - D*xk);
end

function C_RB = coriolis_matrix_rb(xk, mass)
  u = xk(1);
  v = xk(2);
  C_RB = [0,      0,       0, -mass*v;
          0,      0,       0, mass*u;
          0,      0,       0,   0;
          mass*v, -mass*u, 0,   0];
end

function C_A = coriolis_matrix_a(xk, added_mass_matrix)
  u = xk(1);
  v = xk(2);
  x_dot_u = added_mass_matrix(1,1);
  y_dot_v = added_mass_matrix(2,2);
  C_A = [0,          0,         0, y_dot_v*v;
         0,          0,         0, -x_dot_u*u;
         0,          0,         0,   0;
         -y_dot_v*v, x_dot_u*u, 0,   0];
end

function D_v = quadratic_damping_matrix(xk, coefficient)
  u = xk(1);
  v = xk(2);
  w = xk(3);
  r = xk(4);
  D_v = -diag([coefficient.x_abs_u_u*abs(u), ...
              coefficient.y_abs_v_v*abs(v), ...
              coefficient.z_abs_w_w*abs(w), ...
              coefficient.n_abs_r_r*abs(r)]);
end

function xk_plus_1 = rk4(f, xk, tau, dynamic_model, h)
  k1 = feval(f, xk, tau, dynamic_model);
  k2 = feval(f, xk + h/2*k1, tau, dynamic_model);
  k3 = feval(f, xk + h/2*k2, tau, dynamic_model);
  k4 = feval(f, xk + h*k3, tau, dynamic_model);
  xk_plus_1 = xk + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

function plot_robot_states(t, linear_x, nonlinear_x, line_thickness, states_name)
  num_states = size(states_name, 2);

  for i=1:num_states
    subplot(num_states, 1, i)
    plot(t, linear_x(i, :), 'b', 'linewidth', line_thickness, 'DisplayName', 'Linear')
    hold on
    plot(t, nonlinear_x(i, :), 'r', 'linewidth', line_thickness, 'DisplayName', 'Nonlinear')
    if i == 1
      legend('show')
    end
    grid on
    xlabel('Tempo (s)');
    if i < num_states
      ylabel([states_name{i} ' [m/s]']);
    else
      ylabel([states_name{i} ' [rad/s]']);
    end
  end
end
