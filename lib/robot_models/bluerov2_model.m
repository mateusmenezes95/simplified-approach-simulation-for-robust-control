current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

addpath(genpath("../lib")) 
addpath(genpath("../lib/mpc_functions"))
addpath(genpath("../lib/chart_fpatunctions/norms"))
addpath(genpath("../lib/dynamic_models"))

run simulation_parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MANZANILLA, A. et al. Autonomous navigation for unmanned underwater vehicles:
% Real-time experiments using computer vision. IEEE Robotics and Automation Letters,
% v. 4, n. 2, p. 1351–1356, 2019.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

manzanilla_model.mass = 11.0;

manzanilla_model.inertia.zz = 0.16;

manzanilla_model.added_mass_coeficcients.x_dot_u = -5.5;
manzanilla_model.added_mass_coeficcients.y_dot_v = -12.7;
manzanilla_model.added_mass_coeficcients.z_dow_w = -14.57;
manzanilla_model.added_mass_coeficcients.n_dot_r = -0.12;

% The Manzanilla paper does not provides linear and quadratic damping. Hence, it will be used the
% parameters from the Lipenitis model
manzanilla_model.linear_damping_coefficients.x_u = -25.15;
manzanilla_model.linear_damping_coefficients.y_v = -7.364;
manzanilla_model.linear_damping_coefficients.z_w = -17.955;
manzanilla_model.linear_damping_coefficients.m_q = -3.744;

manzanilla_model.quadratic_damping_coefficients.x_abs_u_u = -17.77;
manzanilla_model.quadratic_damping_coefficients.y_abs_v_v = -125.9;
manzanilla_model.quadratic_damping_coefficients.z_abs_w_w = -72.36;
manzanilla_model.quadratic_damping_coefficients.n_abs_r_r = -0.1857;

manzanilla_model.rigid_body_inertia_matrix = get_rigid_body_inertia_matrix(manzanilla_model);
manzanilla_model.added_mass_system_inertia_matrix = get_added_mass_system_inertia_matrix(manzanilla_model);
manzanilla_model.linear_damping_matrix = get_linear_damping_matrix(manzanilla_model);

[discrete_state_space, augmented_state_space] = get_state_space_matrix(manzanilla_model, sampling_period, "Manzanilla Model");
manzanilla_model.discrete_state_space = discrete_state_space;
manzanilla_model.augmented_state_space = augmented_state_space;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Manzanilla Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LIPENITIS, A.; EINARSSON, E. M. MPC control for the BlueROV2 - Theory and
% Implementation. Dissertação (Mestrado) — Aalborg Universitet, 2020.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lipenitis_model.mass = 11.4;

lipenitis_model.inertia.zz = 0.245;

% The Lipenitis paper does not provides added mass paramters. Hence, it will be used the
% parameters from the Lipenitis model
lipenitis_model.added_mass_coeficcients.x_dot_u = -5.5;
lipenitis_model.added_mass_coeficcients.y_dot_v = -12.7;
lipenitis_model.added_mass_coeficcients.z_dow_w = -14.57;
lipenitis_model.added_mass_coeficcients.n_dot_r = -0.12;

lipenitis_model.linear_damping_coefficients.x_u = -25.15;
lipenitis_model.linear_damping_coefficients.y_v = -7.364;
lipenitis_model.linear_damping_coefficients.z_w = -17.955;
lipenitis_model.linear_damping_coefficients.m_q = -3.744;

lipenitis_model.quadratic_damping_coefficients.x_abs_u_u = -17.77;
lipenitis_model.quadratic_damping_coefficients.y_abs_v_v = -125.9;
lipenitis_model.quadratic_damping_coefficients.z_abs_w_w = -72.36;
lipenitis_model.quadratic_damping_coefficients.n_abs_r_r = -0.1857;

lipenitis_model.rigid_body_inertia_matrix = get_rigid_body_inertia_matrix(lipenitis_model);
lipenitis_model.added_mass_system_inertia_matrix = get_added_mass_system_inertia_matrix(lipenitis_model);
lipenitis_model.linear_damping_matrix = get_linear_damping_matrix(lipenitis_model);

[discrete_state_space, augmented_state_space] = get_state_space_matrix(lipenitis_model, sampling_period, "Lipenitis Model");
lipenitis_model.discrete_state_space = discrete_state_space;
lipenitis_model.augmented_state_space = augmented_state_space;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Lipenitis Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BENZON, M. von et al. An open-source benchmark simulator: Control of a bluerov2
% underwater robot. Journal of Marine Science and Engineering, v. 10, n. 12, 2022. ISSN
% 2077-1312. Disponível em: <https://www.mdpi.com/2077-1312/10/12/1898>.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

benzon_model.mass = 13.5;

benzon_model.inertia.zz = 0.37;

benzon_model.added_mass_coeficcients.x_dot_u = -6.36;
benzon_model.added_mass_coeficcients.y_dot_v = -7.12;
benzon_model.added_mass_coeficcients.z_dow_w = -18.68;
benzon_model.added_mass_coeficcients.n_dot_r = -0.22;

benzon_model.linear_damping_coefficients.x_u = -13.7;
benzon_model.linear_damping_coefficients.y_v = -0.0;
benzon_model.linear_damping_coefficients.z_w = -33.0;
benzon_model.linear_damping_coefficients.m_q = -0.0;

benzon_model.quadratic_damping_coefficients.x_abs_u_u = -141.0;
benzon_model.quadratic_damping_coefficients.y_abs_v_v = -217.0;
benzon_model.quadratic_damping_coefficients.z_abs_w_w = -190.0;
benzon_model.quadratic_damping_coefficients.n_abs_r_r = -1.5;

benzon_model.rigid_body_inertia_matrix = get_rigid_body_inertia_matrix(benzon_model);
benzon_model.added_mass_system_inertia_matrix = get_added_mass_system_inertia_matrix(benzon_model);
benzon_model.linear_damping_matrix = get_linear_damping_matrix(benzon_model);

[discrete_state_space, augmented_state_space] = get_state_space_matrix(benzon_model, sampling_period, "Benzon Model");
benzon_model.discrete_state_space = discrete_state_space;
benzon_model.augmented_state_space = augmented_state_space;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Benzon Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

manzanilla_model
lipenitis_model
benzon_model
print_section_description("State Space Model Loaded")

function M_RB = get_rigid_body_inertia_matrix(dynamic_model_parameter)
  M_RB = ...
  [
    dynamic_model_parameter.mass 0 0 0;
    0 dynamic_model_parameter.mass 0 0;
    0 0 dynamic_model_parameter.mass 0;
    0 0 0 dynamic_model_parameter.inertia.zz;
  ];
end

function M_A = get_added_mass_system_inertia_matrix(dynamic_model_parameter)
  M_A = -diag([...
    dynamic_model_parameter.added_mass_coeficcients.x_dot_u ...
    dynamic_model_parameter.added_mass_coeficcients.y_dot_v ...
    dynamic_model_parameter.added_mass_coeficcients.z_dow_w ...
    dynamic_model_parameter.added_mass_coeficcients.n_dot_r
  ]);
end

function D = get_linear_damping_matrix(dynamic_model_parameter)
  D = -diag([...
    dynamic_model_parameter.linear_damping_coefficients.x_u ...
    dynamic_model_parameter.linear_damping_coefficients.y_v ...
    dynamic_model_parameter.linear_damping_coefficients.z_w ...
    dynamic_model_parameter.linear_damping_coefficients.m_q
  ]);
end

function [discrete_state_space, augmented_state_space] = get_state_space_matrix(fossen_model, sampling_period, model_name)
  system_inertia_matrix = fossen_model.rigid_body_inertia_matrix + fossen_model.added_mass_system_inertia_matrix;

  states = {'u', 'v', 'w', 'r'};
  state_vector_size = 4;

  A = -system_inertia_matrix\fossen_model.linear_damping_matrix;
  B = inv(system_inertia_matrix);
  C = eye(state_vector_size);

  robot_continous_model = ss(A, B, C, 0, ...
                            'StateName', states, 'OutputName', states, ...
                            'Name', model_name);

  robot_discrete_model = c2d(robot_continous_model, sampling_period);

  Ad = robot_discrete_model.A;
  Bd = robot_discrete_model.B;
  Cd = robot_discrete_model.C;

  discrete_state_space.Ad = Ad;
  discrete_state_space.Bd = Bd;
  discrete_state_space.Cd = Cd;

  augmented_state_space.Aaug = ...
  [
    Ad    zeros(size(Ad));
    Cd*Ad eye(size(Cd,1),size(Ad,2))
  ];

  augmented_state_space.Baug = ...
  [
    Bd;
    Cd*Bd
  ];

  augmented_state_space.Caug = [zeros(state_vector_size) eye(state_vector_size)];
end
