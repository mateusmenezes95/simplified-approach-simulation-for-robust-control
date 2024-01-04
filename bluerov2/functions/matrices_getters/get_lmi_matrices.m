function [Almi, Blmi, Clmi, Dlmi] = get_lmi_matrices(dynamic_model, dt, np, nu, r_weight, q_weight) 
    % Augmented state-space due the integrator addition
    % [Aaug, Baug, Caug, A, B, C, D] = get_model_matrices(model_params, dt);
  
    dynamic_model.rigid_body_inertia_matrix = get_rigid_body_inertia_matrix(dynamic_model);
    dynamic_model.added_mass_system_inertia_matrix = get_added_mass_system_inertia_matrix(dynamic_model);
    dynamic_model.linear_damping_matrix = get_linear_damping_matrix(dynamic_model);
    
    [discrete_state_space, augmented_state_space] = get_state_space_matrix(dynamic_model, dt, "Dynamic Model");
    dynamic_model.discrete_state_space = discrete_state_space;
    dynamic_model.augmented_state_space = augmented_state_space;

    Aaug = dynamic_model.augmented_state_space.Aaug;
    Baug = dynamic_model.augmented_state_space.Baug;
    Caug = dynamic_model.augmented_state_space.Caug;

    A = dynamic_model.discrete_state_space.Ad;
    B = dynamic_model.discrete_state_space.Bd;
    C = dynamic_model.discrete_state_space.Cd;

    amount_of_states = size(A, 2);
    amount_of_inputs = size(B, 2);
  
    % Matrices to estimate the future states and control signals
    [Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, np, nu);
    [Kw, Kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q_weight, r_weight, np, nu);
    Kmpc = Kmpc(1:amount_of_inputs, :); % Receding horizon control
  
    z = tf('z', dt);
    C_z = -Kmpc*[eye(amount_of_states);(z/(z-1))*C];
    Cz_ss = ss(C_z);
  
    Ac = Cz_ss.A;
    Bc = Cz_ss.B;
    Cc = Cz_ss.C;   
    Dc = Cz_ss.D;

    Almi = [A+B*Dc*C B*Cc zeros(4); Bc*C Ac zeros(4); Dc*C Cc zeros(4)];
    Blmi = [B; zeros(amount_of_inputs); zeros(amount_of_inputs)];
    Clmi = [Dc Cc -eye(4)];
    Dlmi = zeros(4);
end

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
  