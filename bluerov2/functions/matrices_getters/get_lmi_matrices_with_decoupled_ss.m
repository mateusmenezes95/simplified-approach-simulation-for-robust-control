function [Almi, Blmi, Clmi, Dlmi] = get_lmi_matrices_with_decoupled_ss(mass, linear_damping, added_mass, dt, np, nu, r_weight, q_weight)
    [discrete_state_space, augmented_state_space] = get_state_space_matrix(mass, linear_damping, added_mass, dt);
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

    B_times_Cc = B*Cc;

    Almi = [A+B*Dc*C B_times_Cc zeros(size(B_times_Cc, 2)); Bc*C Ac zeros(size(Ac, 2)); Dc*C Cc zeros(size(Cc, 2))];
    Blmi = [B; zeros(amount_of_inputs); zeros(amount_of_inputs)];
    Clmi = [Dc Cc -eye(size(Cc, 1))];
    Dlmi = zeros(1);
end

function [discrete_state_space, augmented_state_space] = get_state_space_matrix(mass, linear_damping, added_mass, sampling_period)
    % Decoupled states
    A = linear_damping/(mass - added_mass);
    B = 1/(mass - added_mass);
    C = 1;

    robot_continous_model = ss(A, B, C, 0);
    robot_discrete_model = c2d(robot_continous_model, sampling_period);

    % We are now working with decoupled states
    state_vector_size = 1;
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
