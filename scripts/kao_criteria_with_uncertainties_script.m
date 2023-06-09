clc
clear

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

addpath(genpath("../lib")) 
addpath(genpath("../lib/mpc_functions"))
addpath(genpath("../lib/chart_functions/norms"))
addpath(genpath("../lib/dynamic_models"))

run simulation_parameters

% =============================================================================
% Nominal model parameters
% =============================================================================

nominal_model.bv = 0.7;  % viscous friction relative to v (N/m/s)
nominal_model.bvn = 0.7;     % viscous friction relative to v n (N/m/s)
nominal_model.bw = 0.011;   % viscous friction relative to ω (N/rad/s)

nominal_model.mass = 1.551;
nominal_model.inertia = 0.0062;  % robots inertial momentum (kg.m 2 )

nominal_model.robot_radius = 0.1;     % robots radius (m)
nominal_model.wheel_radius = 0.0505;  % wheels radius (m)

nominal_model.gear_reduction_rate = 19/1;    % motors gear’s reduction rate
nominal_model.armature_resistance = 1.69;    % armature resistance (Ω)
nominal_model.vel_constant = 0.0059;  % motor velocity constant (V olts/rad/s)
nominal_model.torque_constant = 0.0059;  % torque constant (N.m/A)

robot_param = nominal_model;

% =============================================================================
% Model parameters Uncertainties
% =============================================================================
     
mass_delta = 0.01;
inertia_delta = 0.10;
robot_radius_delta = 0.05;
armature_resistance_delta = 0.3;

uncertainty_mass_vec = [
  nominal_model.mass*(1 - mass_delta), ...
  nominal_model.mass*(1 + mass_delta)
];

uncertainty_inertia_vec = [
  nominal_model.inertia*(1 - inertia_delta), ...
  nominal_model.inertia*(1 + inertia_delta)
];

uncertainty_robot_radius_vec = [
  nominal_model.robot_radius*(1 - robot_radius_delta), ...
  nominal_model.robot_radius*(1 + robot_radius_delta)
];

uncertainty_armature_resistance_vec = [
  nominal_model.armature_resistance*(1 - armature_resistance_delta), ...
  nominal_model.armature_resistance*(1 + armature_resistance_delta)
];

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

q = 91:9:1000;
r = 10000;

lmi_norm_with_uncertainty_vec = zeros(size(q));
lmi_nmax_with_uncertainty_vec = zeros(size(q));

lmi_norm_without_uncertainty_vec = zeros(size(q));
lmi_nmax_without_uncertainty_vec = zeros(size(q));

matlab_norm_without_uncertainty_vec = zeros(size(q));
matlab_nmax_without_uncertainty_vec = zeros(size(q));

matlab_vertex_norm_vec = [zeros(size(q)); zeros(size(q))]';

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
    for i = 1:size(uncertainty_mass_vec, 2)
      for j = 1:size(uncertainty_inertia_vec, 2)
        for k = 1:size(uncertainty_robot_radius_vec, 2)
          for l = 1:size(uncertainty_armature_resistance_vec, 2)
            robot_param.mass = uncertainty_mass_vec(i);
            robot_param.inertia = uncertainty_inertia_vec(j);
            robot_param.robot_radius = uncertainty_robot_radius_vec(k);
            robot_param.armature_resistance = uncertainty_armature_resistance_vec(l);

            [Almi, Blmi, Clmi, Dlmi] = get_lmi_matrices(robot_param, ... 
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

          % Norm computation of the vertex for the closed-loop system
          % closed_loop_ss = ss(Almi, Blmi, Clmi, Dlmi, -1);
          % vertex_norm = norm(closed_loop_ss, inf);
          % matlab_vertex_norm_vec(loop_index, i) = vertex_norm;
          end
        end
      end
    end
  
    objective = mu;
    yalmipdiagnostics = optimize(ineqs, objective, opts);
    mu = value(objective);
    fprintf("Yalmip info for q=%d, r=%d: %s\n", q_sample, r, yalmipdiagnostics.info)

    lmi_norm_with_uncertainty = sqrt(mu);
    lmi_norm_with_uncertainty_vec(loop_index) = lmi_norm_with_uncertainty;

    % Norm computation for the closed-loop system without uncertainty
    [Almi, Blmi, Clmi, Dlmi] = get_lmi_matrices(nominal_model, sampling_period, ...
                                                prediction_horizon, control_horizon, r, q_sample);
    Pd = [];
    Pd = sdpvar(n);
    Gd = [];
    Gd = sdpvar(n, n, 'full');
    mu = 0;
    mu = sdpvar(1);
    ineqs=[];
    ineqs = [ineqs,[Pd Almi*Gd Blmi zeros(n, amount_of_inputs); ...
                    Gd'*Almi' Gd+Gd'-Pd zeros(n, amount_of_inputs) Gd'*Clmi'; ...
                    Blmi' zeros(amount_of_inputs, n) eye(amount_of_inputs) Dlmi'; ...
                    zeros(amount_of_inputs, n) Clmi*Gd Dlmi eye(amount_of_inputs)*mu] >= 0];

    objective = mu;
    yalmipdiagnostics = optimize(ineqs, objective, opts);
    mu = value(objective);

    lmi_norm_without_uncertainty = sqrt(mu);
    lmi_norm_without_uncertainty_vec(loop_index) = lmi_norm_without_uncertainty;

    closed_loop_ss = ss(Almi, Blmi, Clmi, Dlmi, -1);
    matlab_norm_without_uncertainty = norm(closed_loop_ss, inf);
    matlab_norm_without_uncertainty_vec(loop_index) = matlab_norm_without_uncertainty;

    lmi_nmax_with_uncertainty_vec(loop_index) = get_maximum_delay(lmi_norm_with_uncertainty);
    matlab_nmax_without_uncertainty_vec(loop_index) = get_maximum_delay(matlab_norm_without_uncertainty);

    loop_index = loop_index + 1;
end

close(waitbar_fig)

% =============================================================================
figure1 = figure("Name", "Norms by LMI");
% =============================================================================

axes1 = axes('Parent',figure1,...
    'Position',[0.101785714285714 0.102380952380952 0.869642857142857 0.876190476190477]);
hold(axes1,'on');

plot_overlapping_norms(q, 90, lmi_norm_with_uncertainty_vec, ...
                       matlab_norm_without_uncertainty_vec, ...
                       {'With uncertainty', 'Without uncertainty'})

% =============================================================================
figure2 = figure("Name", "Maximum delay allowed by LMI and Matlab");
% =============================================================================

axes1 = axes('Parent',figure2,...
    'Position',[0.0736842105263158 0.113801452784504 0.889473684210526 0.864406779661017]);
hold(axes1,'on');

plot_overlapping_nmax(q, 90, matlab_nmax_without_uncertainty_vec, ...
                      lmi_nmax_with_uncertainty_vec, ...
                      "", {'Without uncertainty', 'With uncertainty'})

function max_delay = get_maximum_delay(norm)
  max_delay = 0;
  if norm < 1
    max_delay = floor(1/norm);
  end
end
