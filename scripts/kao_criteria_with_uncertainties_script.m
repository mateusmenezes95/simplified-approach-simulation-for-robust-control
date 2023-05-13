clc
clear all

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

addpath(genpath("../lib")) 
addpath(genpath("../lib/mpc_functions"))
addpath(genpath("../lib/chart_functions/norms"))

run simulation_parameters

% =============================================================================
% Nominal model parameters
% =============================================================================

nominal_mass = 1.551;
nominal_friction = 0.7;

% =============================================================================
% Model parameters Uncertainties
% =============================================================================

uncertainty_mass_delta = 0.5;
uncertainty_friction_delta = 0.4;

uncertainty_mass_vector = [nominal_mass*(1-uncertainty_mass_delta), ... 
                           nominal_mass*(1+uncertainty_mass_delta)];
uncertainty_friction_vector = [nominal_friction*(1-uncertainty_friction_delta), ...
                              nominal_friction*(1+uncertainty_friction_delta)];

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

Nmax_with_uncertainty_vector = zeros(size(q));
Nmax_without_uncertainty_vector = zeros(size(q));
norms_with_uncertainty = zeros(size(q));
norms_without_uncertainty = zeros(size(q));
vertex_norms = [zeros(size(q)); zeros(size(q))]';

waitbar_fig = waitbar(0, 'Starting LMI computation...');

loop_index= 1;
for q_sample = q
    waitbar((loop_index/size(q,2)), waitbar_fig, ...
            sprintf('Computing H infinity norm for q = %d', q_sample));
    ineqs=[];
    Gd = [];
    Gd = sdpvar(9, 9, 'full');
    mu = 0;
    mu = sdpvar(1);
    for i = 1:size(uncertainty_mass_vector, 2)
      % for j = 1:size(uncertainty_friction_vector, 2)
        [Almi, Blmi, Clmi, Dlmi] = get_lmi_matrices(nominal_mass, ... 
                                                    uncertainty_friction_vector(i), ...
                                                    sampling_period, prediction_horizon, ...
                                                    control_horizon, ...
                                                    r, q_sample);
        % Augmented system size
        n = size(Almi,1);
        Pd = sdpvar(n);

        ineqs = [ineqs,[Pd Almi*Gd Blmi zeros(n, amount_of_inputs); ...
                        Gd'*Almi' Gd+Gd'-Pd zeros(n, amount_of_inputs) Gd'*Clmi'; ...
                        Blmi' zeros(amount_of_inputs, n) eye(amount_of_inputs) Dlmi'; ...
                        zeros(amount_of_inputs, n) Clmi*Gd Dlmi eye(amount_of_inputs)*mu] >= 0];

      % Norm computation for the closed-loop system without uncertainty
      closed_loop_ss = ss(Almi, Blmi, Clmi, Dlmi, -1);
      vertex_norm = norm(closed_loop_ss, inf);
      vertex_norms(loop_index, i) = vertex_norm;
      % end
    end
  
    objective = mu;
    yalmipdiagnostics = optimize(ineqs, objective, opts);
    mu = value(objective);
    fprintf("Yalmip info for q=%d, r=%d: %s\n", q_sample, r, yalmipdiagnostics.info)

    norm_with_uncertainty = sqrt(mu);
    norms_with_uncertainty(loop_index) = norm_with_uncertainty;

    % Norm computation for the closed-loop system without uncertainty
    [Almi, Blmi, Clmi, Dlmi] = get_lmi_matrices(nominal_mass, nominal_friction, sampling_period, ...
                                                prediction_horizon, control_horizon, r, q_sample);
    closed_loop_ss = ss(Almi, Blmi, Clmi, Dlmi, -1);
    norm_without_uncertainty = norm(closed_loop_ss, inf);
    norms_without_uncertainty(loop_index) = norm_without_uncertainty;

    Nmax_with_uncertainty_vector(loop_index) = get_maximum_delay(norm_with_uncertainty);
    Nmax_without_uncertainty_vector(loop_index) = get_maximum_delay(norm_without_uncertainty);

    loop_index = loop_index + 1;
end

close(waitbar_fig)

figure(1)
plot_overlapping_nmax(q, 90, Nmax_without_uncertainty_vector, Nmax_with_uncertainty_vector, ...
                      {'without uncertainty', 'with uncertainty'})

figure(2)
subplot(2,1,1)
legends = {'with uncertainties', 'without uncertainties'};
plot_overlapping_norms(q, 90, norms_with_uncertainty, norms_without_uncertainty, legends)

subplot(2,1,2)
plot_norms(q, 90, norms_with_uncertainty- norms_without_uncertainty, ...
           "Norm by LMI - Norm by Matlab norm function", "H_\infty differences")

figure(3)
legends = {'vertex 1', 'vertex 2'};
plot_overlapping_norms(q, 90, vertex_norms(:,1), vertex_norms(:,2), legends)

function [Aaug, Baug, Caug, Ad, Bd, Cd, Dd] = get_model_matrices(mass, viscous_friction, sampling_period)
  Bv = viscous_friction;  % viscous friction relative to v (N/m/s)
  Bvn = 0.7;     % viscous friction relative to v n (N/m/s)
  Bw = 0.011;   % viscous friction relative to ω (N/rad/s)

  M = mass;
  J = 0.0062;  % robots inertial momentum (kg.m 2 )

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

function [Almi, Blmi, Clmi, Dlmi] = get_lmi_matrices(mass, friction, dt, np, nu, r_weight, q_weight) 
  % Augmented state-space due the integrator addition
  [Aaug, Baug, Caug, A, B, C, D] = get_model_matrices(mass, friction, dt);

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

  Almi = [A+B*Dc*C B*Cc zeros(3); Bc*C Ac zeros(3); Dc*C Cc zeros(3)];
  Blmi = [B; zeros(3); zeros(3)];
  Clmi = [Dc Cc -eye(3)];
  Dlmi = zeros(3);
end

function max_delay = get_maximum_delay(norm)
  max_delay = 0;
  if norm < 1
    max_delay = floor(1/norm);
  end
end
