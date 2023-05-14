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
     
uncertainty_vec = zeros(2);
uncertainty_option = 'robot_radius';
delta = 0.3;

switch uncertainty_option
  case 'mass'
    nominal_value = nominal_model.mass;
  case 'friction'
    nominal_value = nominal_model.bv;
  case 'armature_resistance'
    nominal_value = nominal_model.armature_resistance;
  case 'robot_radius'
    nominal_value = nominal_model.robot_radius;
  case 'wheel_radius'
    nominal_value = nominal_model.wheel_radius;
  otherwise
      disp('Unknown uncertainty option');
      return
end

uncertainty_vec = [nominal_value*(1 - delta), nominal_value*(1 + delta)];

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
    for i = 1:size(uncertainty_vec, 2)
      % for j = 1:size(uncertainty_vec, 2)

        switch uncertainty_option
          case 'mass'
            robot_param.mass = uncertainty_vec(i);
          case 'friction'
            robot_param.bv = uncertainty_vec(i);
          case 'armature_resistance'
            robot_param.armature_resistance = uncertainty_vec(i);
          case 'robot_radius'
            robot_param.robot_radius = uncertainty_vec(i);
        end

        % if j == 1
        %   robot_param.armature_resistance = robot_param.armature_resistance*(1 - 0.2);
        % else
        %   robot_param.armature_resistance = robot_param.armature_resistance*(1 + 0.2);
        % end

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
      closed_loop_ss = ss(Almi, Blmi, Clmi, Dlmi, -1);
      vertex_norm = norm(closed_loop_ss, inf);
      matlab_vertex_norm_vec(loop_index, i) = vertex_norm;
      % end
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
figure("Name", "Vertex Norms")
% =============================================================================

subplot(2,2,1)
plot_norms(q, 90, matlab_vertex_norm_vec(:,1), "Vertex 1", "||H||_\infty")

subplot(2,2,2)
plot_norms(q, 90, matlab_vertex_norm_vec(:,2), "Vertex 2", "||H||_\infty")

subplot(2,2,3)
plot_norms(q, 90, matlab_vertex_norm_vec(:,1)- matlab_vertex_norm_vec(:,2), ...
           "Vertex 1 - Vertex 2", "||H||_\infty differences")

subplot(2,2,4)
max_vertex_norm = max(matlab_vertex_norm_vec, [], 2)';
plot_norms(q, 90, max_vertex_norm, "Max norm of the two vertices", "||H||_\infty")

% =============================================================================
figure("Name", "Norms by LMI")
% =============================================================================

subplot(2,2,1)
plot_norms(q, 90, lmi_norm_with_uncertainty_vec, ...
           "||H||_\infty by LMI with uncertainty", "||H||_\infty")

subplot(2,2,2)
plot_norms(q, 90, lmi_norm_without_uncertainty_vec, ...
           "||H||_\infty by LMI without uncertainty", "||H||_\infty")

subplot(2,2,3)
plot_norms(q, 90, lmi_norm_with_uncertainty_vec - lmi_norm_without_uncertainty_vec, ...
           "||H||_\infty by LMI with uncertainty - ||H||_\infty by LMI without uncertainty", ...
           "||H||_\infty")

subplot(2,2,4)
plot_norms(q, 90, lmi_norm_with_uncertainty_vec - max_vertex_norm, ...
           "||H||_\infty by LMI with uncertainty - max ||H||_\infty by Matlab of the two vertices", ...
           "||H||_\infty norm")

% =============================================================================
figure("Name", "Norms of Nominal Model by LMI and Matlab")
% =============================================================================

subplot(3,1,1)
plot_norms(q, 90, matlab_norm_without_uncertainty_vec, ...
           "||H||_\infty by Matlab for the nominal model", "||H||_\infty")

subplot(3,1,2)
plot_norms(q, 90, lmi_norm_without_uncertainty_vec, ...
           "||H||_\infty by LMI for the nominal model", "||H||_\infty")

subplot(3,1,3)
plot_norms(q, 90, matlab_norm_without_uncertainty_vec - lmi_norm_without_uncertainty_vec, ...
           "||H||_\infty by Matlab - ||H||_\infty by LMI for the nominal model", ...
           "||H||_\infty")

% =============================================================================
figure("Name", "Maximum delay allowed by LMI and Matlab")
% =============================================================================

subplot(3,1,1)
plot_nmax(q, 90, matlab_nmax_without_uncertainty_vec, ...
          "Maximum delay allowed by Matlab ||H||_\infty  for the nominal model")

subplot(3,1,2)
plot_nmax(q, 90, lmi_nmax_with_uncertainty_vec, ...
          "Maximum delay allowed by LMI ||H||_\infty  for the model with uncertainty")

subplot(3,1,3)
plot_overlapping_nmax(q, 90, matlab_nmax_without_uncertainty_vec, ...
                      lmi_nmax_with_uncertainty_vec, ...
                      "Comparison between the maximum delay allowed by Matlab and LMI", ...
                      {'Without uncertainty', 'With uncertainty'})

function max_delay = get_maximum_delay(norm)
  max_delay = 0;
  if norm < 1
    max_delay = floor(1/norm);
  end
end
