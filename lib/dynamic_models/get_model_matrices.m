function [Aaug, Baug, Caug, ...
    Ad, Bd, Cd, Dd] = get_model_matrices(model_params_struct, sampling_period)

    Bv = model_params_struct.bv;
    Bvn = model_params_struct.bvn;
    Bw = model_params_struct.bw;

    M = model_params_struct.mass;
    J = model_params_struct.inertia;

    L = model_params_struct.robot_radius;
    wr = model_params_struct.wheel_radius;

    nr = model_params_struct.gear_reduction_rate;
    Ra = model_params_struct.armature_resistance;
    Kv = model_params_struct.vel_constant;
    Kt = model_params_struct.torque_constant;

    % Plant dynamic continuous matrix
    a11 = (-(3*(nr^2)*Kt*Kv)/(2*M*Ra*(wr^2)))-(Bv/M);
    a22 = a11;
    a33 = (-(3*(L^2)*(nr^2)*Kt*Kv)/(J*Ra*(wr^2)))-(Bw/J);

    A = ...
    [
    a11  0    0;
    0    a22  0;
    0    0    a33;
    ];

    B = ...
    [
    0     sqrt(3)/(2*M)   -sqrt(3)/(2*M)
    -1/M  1/(2*M)         1/(2*M)
    L/J  L/J           L/J
    ];
    B = ((nr*Kt)/(Ra*wr))*B;

    C = eye(3);

    states = {'v', 'vn', 'w'};
    state_vector_size = 3;

    robot_continous_model = ss(A, B, C, 0, ...
                        'StateName', states, 'OutputName', states, ...
                        'Name', 'Robot Model');

    robot_discrete_model = c2d(robot_continous_model, sampling_period);

    Ad = robot_discrete_model.A;
    Bd = robot_discrete_model.B;
    Cd = robot_discrete_model.C;
    Dd = robot_discrete_model.D;

    Aaug = ...
    [
    Ad    zeros(size(Ad));
    Cd*Ad eye(size(Cd,1),size(Ad,2))
    ];

    Baug = ...
    [
    Bd;
    Cd*Bd
    ];

    Caug = [zeros(state_vector_size) eye(state_vector_size)];
end
