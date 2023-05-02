clc
clear all

addpath(genpath("../lib")) 
addpath(genpath("../lib/mpc_functions")) 

run simulation_parameters

% =============================================================================
% Nominal model parameters
% =============================================================================

nominal_mass = 1.551;
nominal_inertia = 0.0062;

% =============================================================================
% Model parameters Uncertainties
% =============================================================================

uncertainty_delta = 0.2;

uncertainty_lower_bound = (1 - uncertainty_delta);
uncertainty_upper_bound = (1 + uncertainty_delta);

uncertainty_mass_vector = [nominal_mass*uncertainty_lower_bound, ... 
                           nominal_mass*uncertainty_upper_bound];
uncertainty_inertia_vector = [nominal_inertia*uncertainty_lower_bound, ...
                              nominal_inertia*uncertainty_upper_bound];

% =============================================================================
% State-space variables size
% =============================================================================

amount_of_outputs = 3;
amount_of_inputs = 3;
amount_of_states = 3;

% =============================================================================
% Solver options
% =============================================================================

opts = sdpsettings;
opts.savesolveroutput = 1;
opts.verbose = 0;
opts.solver = 'sdpt3';

% =============================================================================
% MPC tunning
% =============================================================================

q = 1:9:1000;
r = 10000;

Nmax_vector = zeros(size(q));
h_infnty_norm_vector = zeros(size(q));

loop_index= 1;
for q_sample = q
    fprintf('Computing H infinity norm for q = %d. Progress %.2f\n%', q_sample, (loop_index/size(q,2))*100);
    % norms_by_matlab = [];
    ineqs=[];
    for i = 1:size(uncertainty_mass_vector, 2)
      for j = 1:size(uncertainty_inertia_vector, 2)
        % Augmented state-space due the integrator addition
        [Aaug, Baug, Caug, A, B, C, D] = get_model_matrices(uncertainty_mass_vector(i), ...
                                uncertainty_inertia_vector(j), sampling_period);

        % Matrices to estimate the future states and control signals
        [Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
        [Kw, Kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q_sample, r, prediction_horizon, control_horizon);
        Kmpc = Kmpc(1:amount_of_inputs, :); % Receding horizon control

        z = tf('z', sampling_period);
        C_z = -Kmpc*[eye(amount_of_states);(z/(z-1))*C];
        Cz_ss = ss(C_z);

        Ac = Cz_ss.A;
        Bc = Cz_ss.B;
        Cc = Cz_ss.C;
        Dc = Cz_ss.D;

        Almi = [A+B*Dc*C B*Cc zeros(3); Bc*C Ac zeros(3); Dc*C Cc zeros(3)];
        Blmi = [B; zeros(3); zeros(3)];
        Clmi = [Dc Cc -eye(3)];
        Dlmi = zeros(3);

        n = size(Almi,1); % dimensão do sistema aumentado
        Pd = sdpvar(n);
        Gd = sdpvar(n, n, 'full');
        mu = sdpvar(1);

        ineqs = [ineqs,[Pd Almi*Gd Blmi zeros(n, amount_of_inputs);...
                        Gd'*Almi' Gd+Gd'-Pd zeros(n, amount_of_inputs) Gd'*Clmi'; ...
                        Blmi' zeros(amount_of_inputs, n) eye(amount_of_inputs) Dlmi';...
                        zeros(amount_of_inputs, n) Clmi*Gd Dlmi eye(amount_of_inputs)*mu] >= 0];

        % sys1=ss(Almi,Blmi,Clmi,Dlmi,-1);
        % norms_by_matlab = [norms_by_matlab, norm(sys1,2)];
      end
    end

    objective = mu;
    yalmipdiagnostics = optimize(ineqs, objective, opts);
    mu = value(objective);
    fprintf("Yalmip info for q=%d, r=%d: %s\n", q_sample, r, yalmipdiagnostics.info)

    h_infnty_norm = sqrt(mu);
    h_infnty_norm_vector(loop_index) = h_infnty_norm;

    Nmax = 0;
    if h_infnty_norm < 1
      Nmax = floor(1/h_infnty_norm);
    end
    Nmax_vector(loop_index) = Nmax;

    loop_index = loop_index + 1;
end

figure(1)
stem(q, Nmax_vector, "Marker",".")
grid on
xlabel('q')
ylabel('N_{max}')
xlim([q(2) max(q)])
yticks(0:1:Nmax_vector(2))
xticks(q(2):90:max(q))
if max(Nmax_vector) == 0
    ylim([0 1])
    yticks([0 1])
else
    ylim([0 Nmax_vector(2)])
end

figure(2)
stem(q, h_infnty_norm_vector, "Marker",".")
grid on
xlabel('q')
ylabel('Norm_\infty')
xlim([q(2) max(q)])
xticks(q(2):90:max(q))

function [Aaug, Baug, Caug, Ad, Bd, Cd, Dd] = get_model_matrices(mass, inertia, sampling_period)
  Bv = 0.7;      % viscous friction relative to v (N/m/s)
  Bvn = 0.7;     % viscous friction relative to v n (N/m/s)
  Bw = 0.011;   % viscous friction relative to ω (N/rad/s)

  M = mass;
  J = inertia;  % robots inertial momentum (kg.m 2 )

  L = 0.1;     % robots radius (m)
  wr = 0.0505;  % wheels radius (m)

  nr = 19/1;    % motors gear’s reduction rate
  Ra = 1.69;    % armature resistance (Ω)
  Kv = 0.0059;  % motor velocity constant (V olts/rad/s)
  Kt = 0.0059;  % torque constant (N.m/A)

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
