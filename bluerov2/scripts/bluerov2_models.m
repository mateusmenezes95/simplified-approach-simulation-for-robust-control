current_script_path = fileparts(mfilename('fullpath'));
addpath(genpath(current_script_path  + "/../../lib/utils")) 

run bluerov2_simulation_parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MANZANILLA, A. et al. Autonomous navigation for unmanned underwater vehicles:
% Real-time experiments using computer vision. IEEE Robotics and Automation Letters,
% v. 4, n. 2, p. 1351–1356, 2019.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
manzanilla_model.name = "Manzanilla";
manzanilla_model.mass = 11.0;

manzanilla_model.inertia.zz = 0.16;

manzanilla_model.added_mass_coeficcients.x_dot_u = -5.5;
manzanilla_model.added_mass_coeficcients.y_dot_v = -12.7;
manzanilla_model.added_mass_coeficcients.z_dot_w = -14.57;
manzanilla_model.added_mass_coeficcients.n_dot_r = -0.12;

% The Manzanilla paper does not provides linear and quadratic damping. Hence, it will be used the
% parameters from the Lipenitis model
manzanilla_model.linear_damping_coefficients.x_u = -25.15;
manzanilla_model.linear_damping_coefficients.y_v = -7.364;
manzanilla_model.linear_damping_coefficients.z_w = -17.955;
manzanilla_model.linear_damping_coefficients.n_r = -3.744;

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

manzanilla_model.color = "r";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Manzanilla Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LIPENITIS, A.; EINARSSON, E. M. MPC control for the BlueROV2 - Theory and
% Implementation. Dissertação (Mestrado) — Aalborg Universitet, 2020.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lipenitis_model.name = "Lipenitis";
lipenitis_model.mass = 11.4;

lipenitis_model.inertia.zz = 0.245;

% The Lipenitis paper does not provides added mass paramters. Hence, it will be used the
% parameters from the Lipenitis model
lipenitis_model.added_mass_coeficcients.x_dot_u = -5.5;
lipenitis_model.added_mass_coeficcients.y_dot_v = -12.7;
lipenitis_model.added_mass_coeficcients.z_dot_w = -14.57;
lipenitis_model.added_mass_coeficcients.n_dot_r = -0.12;

lipenitis_model.linear_damping_coefficients.x_u = -25.15;
lipenitis_model.linear_damping_coefficients.y_v = -7.364;
lipenitis_model.linear_damping_coefficients.z_w = -17.955;
lipenitis_model.linear_damping_coefficients.n_r = -3.744;

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

lipenitis_model.color = "b";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Lipenitis Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BENZON, M. von et al. An open-source benchmark simulator: Control of a bluerov2
% underwater robot. Journal of Marine Science and Engineering, v. 10, n. 12, 2022. ISSN
% 2077-1312. Disponível em: <https://www.mdpi.com/2077-1312/10/12/1898>.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
benzon_model.name = "Benzon";
benzon_model.mass = 13.5;

benzon_model.inertia.zz = 0.37;

benzon_model.added_mass_coeficcients.x_dot_u = -6.36;
benzon_model.added_mass_coeficcients.y_dot_v = -7.12;
benzon_model.added_mass_coeficcients.z_dot_w = -18.68;
benzon_model.added_mass_coeficcients.n_dot_r = -0.22;

% Benzon reported zeroed linear damping coefficients for sway and yaw DOF. This causes an integrator behavior
% when simulating the linearized model. Hence, I borrowed these parameters from lipenitis model
benzon_model.linear_damping_coefficients.x_u = -13.7;
benzon_model.linear_damping_coefficients.y_v = lipenitis_model.linear_damping_coefficients.y_v;
benzon_model.linear_damping_coefficients.z_w = -33.0;
benzon_model.linear_damping_coefficients.n_r = lipenitis_model.linear_damping_coefficients.n_r;

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

benzon_model.color = "g";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Benzon Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This model represents the lower bound of the system. It is the model with the smallest parameters
% among the three models.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lower_model.name = "Lower Model";
lower_model.mass = min([manzanilla_model.mass, lipenitis_model.mass, benzon_model.mass]);

lower_model.inertia.zz = min([manzanilla_model.inertia.zz, lipenitis_model.inertia.zz, benzon_model.inertia.zz]);

lower_model.added_mass_coeficcients.x_dot_u = min([manzanilla_model.added_mass_coeficcients.x_dot_u, ...
                                                   lipenitis_model.added_mass_coeficcients.x_dot_u, ...
                                                   benzon_model.added_mass_coeficcients.x_dot_u]);
lower_model.added_mass_coeficcients.y_dot_v = min([manzanilla_model.added_mass_coeficcients.y_dot_v, ...
                                                   lipenitis_model.added_mass_coeficcients.y_dot_v, ...
                                                   benzon_model.added_mass_coeficcients.y_dot_v]);
lower_model.added_mass_coeficcients.z_dot_w = min([manzanilla_model.added_mass_coeficcients.z_dot_w, ...
                                                    lipenitis_model.added_mass_coeficcients.z_dot_w, ...
                                                    benzon_model.added_mass_coeficcients.z_dot_w]);
lower_model.added_mass_coeficcients.n_dot_r = min([manzanilla_model.added_mass_coeficcients.n_dot_r, ...
                                                    lipenitis_model.added_mass_coeficcients.n_dot_r, ...
                                                    benzon_model.added_mass_coeficcients.n_dot_r]);

lower_model.linear_damping_coefficients.x_u = min([manzanilla_model.linear_damping_coefficients.x_u, ...
                                                    lipenitis_model.linear_damping_coefficients.x_u, ...
                                                    benzon_model.linear_damping_coefficients.x_u]);
lower_model.linear_damping_coefficients.y_v = min([manzanilla_model.linear_damping_coefficients.y_v, ...
                                                    lipenitis_model.linear_damping_coefficients.y_v, ...
                                                    benzon_model.linear_damping_coefficients.y_v]);
lower_model.linear_damping_coefficients.z_w = min([manzanilla_model.linear_damping_coefficients.z_w, ...
                                                    lipenitis_model.linear_damping_coefficients.z_w, ...
                                                    benzon_model.linear_damping_coefficients.z_w]);
lower_model.linear_damping_coefficients.n_r = min([manzanilla_model.linear_damping_coefficients.n_r, ...
                                                    lipenitis_model.linear_damping_coefficients.n_r, ...
                                                    benzon_model.linear_damping_coefficients.n_r]);

lower_model.quadratic_damping_coefficients.x_abs_u_u = min([manzanilla_model.quadratic_damping_coefficients.x_abs_u_u, ...
                                                            lipenitis_model.quadratic_damping_coefficients.x_abs_u_u, ...
                                                            benzon_model.quadratic_damping_coefficients.x_abs_u_u]);
lower_model.quadratic_damping_coefficients.y_abs_v_v = min([manzanilla_model.quadratic_damping_coefficients.y_abs_v_v, ...
                                                            lipenitis_model.quadratic_damping_coefficients.y_abs_v_v, ...
                                                            benzon_model.quadratic_damping_coefficients.y_abs_v_v]);
lower_model.quadratic_damping_coefficients.z_abs_w_w = min([manzanilla_model.quadratic_damping_coefficients.z_abs_w_w, ...
                                                            lipenitis_model.quadratic_damping_coefficients.z_abs_w_w, ...
                                                            benzon_model.quadratic_damping_coefficients.z_abs_w_w]);
lower_model.quadratic_damping_coefficients.n_abs_r_r = min([manzanilla_model.quadratic_damping_coefficients.n_abs_r_r, ...
                                                            lipenitis_model.quadratic_damping_coefficients.n_abs_r_r, ...
                                                            benzon_model.quadratic_damping_coefficients.n_abs_r_r]);

lower_model.rigid_body_inertia_matrix = get_rigid_body_inertia_matrix(lower_model);
lower_model.added_mass_system_inertia_matrix = get_added_mass_system_inertia_matrix(lower_model);
lower_model.linear_damping_matrix = get_linear_damping_matrix(lower_model);

[discrete_state_space, augmented_state_space] = get_state_space_matrix(lower_model, sampling_period, lower_model.name);
lower_model.discrete_state_space = discrete_state_space;
lower_model.augmented_state_space = augmented_state_space;

lower_model.color = "y";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Lower Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This model represents the upper bound of the system. It is the model with the highest parameters
% among the three models.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

upper_model.name = "Upper Model";

upper_model.mass = max([manzanilla_model.mass, lipenitis_model.mass, benzon_model.mass]);

upper_model.inertia.zz = max([manzanilla_model.inertia.zz, lipenitis_model.inertia.zz, benzon_model.inertia.zz]);

upper_model.added_mass_coeficcients.x_dot_u = max([manzanilla_model.added_mass_coeficcients.x_dot_u, ...
                                                   lipenitis_model.added_mass_coeficcients.x_dot_u, ...
                                                   benzon_model.added_mass_coeficcients.x_dot_u]);
upper_model.added_mass_coeficcients.y_dot_v = max([manzanilla_model.added_mass_coeficcients.y_dot_v, ...
                                                    lipenitis_model.added_mass_coeficcients.y_dot_v, ...
                                                    benzon_model.added_mass_coeficcients.y_dot_v]);
upper_model.added_mass_coeficcients.z_dot_w = max([manzanilla_model.added_mass_coeficcients.z_dot_w, ...
                                                    lipenitis_model.added_mass_coeficcients.z_dot_w, ...
                                                    benzon_model.added_mass_coeficcients.z_dot_w]);
upper_model.added_mass_coeficcients.n_dot_r = max([manzanilla_model.added_mass_coeficcients.n_dot_r, ...
                                                    lipenitis_model.added_mass_coeficcients.n_dot_r, ...
                                                    benzon_model.added_mass_coeficcients.n_dot_r]);

upper_model.linear_damping_coefficients.x_u = max([manzanilla_model.linear_damping_coefficients.x_u, ...
                                                    lipenitis_model.linear_damping_coefficients.x_u, ...
                                                    benzon_model.linear_damping_coefficients.x_u]);
upper_model.linear_damping_coefficients.y_v = max([manzanilla_model.linear_damping_coefficients.y_v, ...
                                                    lipenitis_model.linear_damping_coefficients.y_v, ...
                                                    benzon_model.linear_damping_coefficients.y_v]);
upper_model.linear_damping_coefficients.z_w = max([manzanilla_model.linear_damping_coefficients.z_w, ...
                                                    lipenitis_model.linear_damping_coefficients.z_w, ...
                                                    benzon_model.linear_damping_coefficients.z_w]);
upper_model.linear_damping_coefficients.n_r = max([manzanilla_model.linear_damping_coefficients.n_r, ...
                                                    lipenitis_model.linear_damping_coefficients.n_r, ...
                                                    benzon_model.linear_damping_coefficients.n_r]);

upper_model.quadratic_damping_coefficients.x_abs_u_u = max([manzanilla_model.quadratic_damping_coefficients.x_abs_u_u, ...
                                                            lipenitis_model.quadratic_damping_coefficients.x_abs_u_u, ...
                                                            benzon_model.quadratic_damping_coefficients.x_abs_u_u]);
upper_model.quadratic_damping_coefficients.y_abs_v_v = max([manzanilla_model.quadratic_damping_coefficients.y_abs_v_v, ...
                                                            lipenitis_model.quadratic_damping_coefficients.y_abs_v_v, ...
                                                            benzon_model.quadratic_damping_coefficients.y_abs_v_v]);
upper_model.quadratic_damping_coefficients.z_abs_w_w = max([manzanilla_model.quadratic_damping_coefficients.z_abs_w_w, ...
                                                            lipenitis_model.quadratic_damping_coefficients.z_abs_w_w, ...
                                                            benzon_model.quadratic_damping_coefficients.z_abs_w_w]);
upper_model.quadratic_damping_coefficients.n_abs_r_r = max([manzanilla_model.quadratic_damping_coefficients.n_abs_r_r, ...
                                                            lipenitis_model.quadratic_damping_coefficients.n_abs_r_r, ...
                                                            benzon_model.quadratic_damping_coefficients.n_abs_r_r]);

upper_model.rigid_body_inertia_matrix = get_rigid_body_inertia_matrix(upper_model);
upper_model.added_mass_system_inertia_matrix = get_added_mass_system_inertia_matrix(upper_model);
upper_model.linear_damping_matrix = get_linear_damping_matrix(upper_model);

[discrete_state_space, augmented_state_space] = get_state_space_matrix(upper_model, sampling_period, upper_model.name);
upper_model.discrete_state_space = discrete_state_space;
upper_model.augmented_state_space = augmented_state_space;

upper_model.color = "c"

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Upper Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This model represents the nominal model of the system. It is the model with the average parameters
% among the lower and upper bounds
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nominal_model.name = "Nominal Model";

nominal_model.mass = (lower_model.mass + upper_model.mass)/2;

nominal_model.inertia.zz = (lower_model.inertia.zz + upper_model.inertia.zz)/2;

nominal_model.added_mass_coeficcients.x_dot_u = (lower_model.added_mass_coeficcients.x_dot_u + ...
                                                 upper_model.added_mass_coeficcients.x_dot_u)/2;
nominal_model.added_mass_coeficcients.y_dot_v = (lower_model.added_mass_coeficcients.y_dot_v + ...
                                                  upper_model.added_mass_coeficcients.y_dot_v)/2;
nominal_model.added_mass_coeficcients.z_dot_w = (lower_model.added_mass_coeficcients.z_dot_w + ...
                                                  upper_model.added_mass_coeficcients.z_dot_w)/2;
nominal_model.added_mass_coeficcients.n_dot_r = (lower_model.added_mass_coeficcients.n_dot_r + ...
                                                  upper_model.added_mass_coeficcients.n_dot_r)/2;

nominal_model.linear_damping_coefficients.x_u = (lower_model.linear_damping_coefficients.x_u + ...
                                                  upper_model.linear_damping_coefficients.x_u)/2;
nominal_model.linear_damping_coefficients.y_v = (lower_model.linear_damping_coefficients.y_v + ...
                                                  upper_model.linear_damping_coefficients.y_v)/2;
nominal_model.linear_damping_coefficients.z_w = (lower_model.linear_damping_coefficients.z_w + ...
                                                  upper_model.linear_damping_coefficients.z_w)/2;
nominal_model.linear_damping_coefficients.n_r = (lower_model.linear_damping_coefficients.n_r + ...
                                                  upper_model.linear_damping_coefficients.n_r)/2;

nominal_model.quadratic_damping_coefficients.x_abs_u_u = (lower_model.quadratic_damping_coefficients.x_abs_u_u + ...
                                                          upper_model.quadratic_damping_coefficients.x_abs_u_u)/2;
nominal_model.quadratic_damping_coefficients.y_abs_v_v = (lower_model.quadratic_damping_coefficients.y_abs_v_v + ...
                                                          upper_model.quadratic_damping_coefficients.y_abs_v_v)/2;
nominal_model.quadratic_damping_coefficients.z_abs_w_w = (lower_model.quadratic_damping_coefficients.z_abs_w_w + ...
                                                          upper_model.quadratic_damping_coefficients.z_abs_w_w)/2;
nominal_model.quadratic_damping_coefficients.n_abs_r_r = (lower_model.quadratic_damping_coefficients.n_abs_r_r + ...
                                                          upper_model.quadratic_damping_coefficients.n_abs_r_r)/2;

nominal_model.rigid_body_inertia_matrix = get_rigid_body_inertia_matrix(nominal_model);
nominal_model.added_mass_system_inertia_matrix = get_added_mass_system_inertia_matrix(nominal_model);
nominal_model.linear_damping_matrix = get_linear_damping_matrix(nominal_model);

[discrete_state_space, augmented_state_space] = get_state_space_matrix(nominal_model, sampling_period, nominal_model.name);
nominal_model.discrete_state_space = discrete_state_space;
nominal_model.augmented_state_space = augmented_state_space;

nominal_model.color = "m";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Upper Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

manzanilla_model
lipenitis_model
benzon_model
lower_model
upper_model
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
    dynamic_model_parameter.added_mass_coeficcients.z_dot_w ...
    dynamic_model_parameter.added_mass_coeficcients.n_dot_r
  ]);
end

function D = get_linear_damping_matrix(dynamic_model_parameter)
  D = -diag([...
    dynamic_model_parameter.linear_damping_coefficients.x_u ...
    dynamic_model_parameter.linear_damping_coefficients.y_v ...
    dynamic_model_parameter.linear_damping_coefficients.z_w ...
    dynamic_model_parameter.linear_damping_coefficients.n_r
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
