clc
clear

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

addpath(genpath("."))
addpath(genpath("../../lib/utils")) 
addpath(genpath("../../lib/mpc_functions"))
addpath(genpath("../../lib/charts_functions/norms"))
addpath(genpath("../functions/matrices_getters"))

run bluerov2_simulation_parameters
run bluerov2_models

degrees_of_freedom = string(["Surge", "Sway", "Heave", "Yaw"]);

uncertainty_mass_or_inertia_vec = [
  [lower_model.mass, upper_model.mass]; ...               % For surge dof
  [lower_model.mass, upper_model.mass]; ...               % For sway dof
  [lower_model.mass, upper_model.mass]; ...               % For heave dof
  [lower_model.inertia.zz, upper_model.inertia.zz]; ...   % For yaw dof
];

uncertainty_linear_damping_vec = [
  [lower_model.linear_damping_coefficients.x_u, upper_model.linear_damping_coefficients.x_u]; ...  % For surge dof
  [lower_model.linear_damping_coefficients.y_v, upper_model.linear_damping_coefficients.y_v]; ...  % For sway dof
  [lower_model.linear_damping_coefficients.z_w, upper_model.linear_damping_coefficients.z_w]; ...  % For heave dof
  [lower_model.linear_damping_coefficients.n_r, upper_model.linear_damping_coefficients.n_r]; ...  % For yaw dof
];

uncertainty_added_mass_vec = [
  [lower_model.added_mass_coeficcients.x_dot_u, upper_model.added_mass_coeficcients.x_dot_u]; ...  % For surge dof
  [lower_model.added_mass_coeficcients.y_dot_v, upper_model.added_mass_coeficcients.y_dot_v]; ...  % For sway dof
  [lower_model.added_mass_coeficcients.z_dot_w, upper_model.added_mass_coeficcients.z_dot_w]; ...  % For heave dof
  [lower_model.added_mass_coeficcients.n_dot_r, upper_model.added_mass_coeficcients.n_dot_r]; ...  % For yaw dof
];

% =============================================================================
% State-space variables size
% =============================================================================

amount_of_outputs = 1;
amount_of_inputs = 1;
amount_of_states = 1;
amount_of_decoupled_states = size(uncertainty_mass_or_inertia_vec, 1);

% =============================================================================
% Solver options
% =============================================================================

opts = sdpsettings;
opts.savesolveroutput = 1;
opts.verbose = 0;
opts.solver = 'sdpt3';
opt.allownonconvex = 0;

% =============================================================================
% MPC tunning
% =============================================================================

q = 7111:9:8000;
r = 200;

lmi_norm_with_uncertainty_vec = zeros(amount_of_decoupled_states, size(q, 2));
lmi_nmax_with_uncertainty_vec = zeros(amount_of_decoupled_states, size(q, 2));

lmi_norm_without_uncertainty_vec = zeros(amount_of_decoupled_states, size(q, 2));
lmi_nmax_without_uncertainty_vec = zeros(amount_of_decoupled_states, size(q, 2));

matlab_norm_without_uncertainty_vec = zeros(amount_of_decoupled_states, size(q, 2));
matlab_nmax_without_uncertainty_vec = zeros(amount_of_decoupled_states, size(q, 2));

for ss_num = 1:amount_of_decoupled_states
  print_section_description("Processing DoF: " + degrees_of_freedom(ss_num))

  waitbar_fig = waitbar(0, 'Starting LMI computation...');
  loop_index= 1;
  for q_sample = q
      waitbar((loop_index/size(q,2)), waitbar_fig, ...
              sprintf('Computing H infinity norm for q = %d', q_sample));
      ineqs=[];
      Gd = [];
      Gd = sdpvar(3, 3, 'full');
      mu = 0;
      mu = sdpvar(1);
      for i = 1:size(uncertainty_mass_or_inertia_vec, 2)
        for j = 1:size(uncertainty_linear_damping_vec, 2)
          for k = 1:size(uncertainty_added_mass_vec, 2)
            mass_or_inertia = uncertainty_mass_or_inertia_vec(ss_num, i);
            linear_damping = uncertainty_linear_damping_vec(ss_num, j);
            added_mass = uncertainty_added_mass_vec(ss_num, k);
            [Almi, Blmi, Clmi, Dlmi] = get_lmi_matrices_with_decoupled_ss(...
                                                        mass_or_inertia, ...
                                                        linear_damping, ...
                                                        added_mass, ...
                                                        sampling_period, prediction_horizon, ...
                                                        control_horizon, r, q_sample);
            % Augmented system size
            n = size(Almi,1);
            Pd = sdpvar(n);

            ineqs = [ineqs,[Pd Almi*Gd Blmi zeros(n, amount_of_inputs); ...
                            Gd'*Almi' Gd+Gd'-Pd zeros(n, amount_of_inputs) Gd'*Clmi'; ...
                            Blmi' zeros(amount_of_inputs, n) eye(amount_of_inputs) Dlmi'; ...
                            zeros(amount_of_inputs, n) Clmi*Gd Dlmi eye(amount_of_inputs)*mu] >= 0];
          end % end for k loop
        end % end for j loop
      end % end for i loop
    
      objective = mu;
      yalmipdiagnostics = optimize(ineqs, objective, opts);
      mu = value(objective);
      fprintf("Yalmip info for q=%d, r=%d: %s\n", q_sample, r, yalmipdiagnostics.info)

      lmi_norm_with_uncertainty = sqrt(mu);
      lmi_norm_with_uncertainty_vec(ss_num, loop_index) = lmi_norm_with_uncertainty;

      [Almi, Blmi, Clmi, Dlmi] = get_lmi_matrices_with_decoupled_ss(nominal_model.mass,...
                                                                    nominal_model.linear_damping_coefficients.x_u, ...
                                                                    nominal_model.added_mass_coeficcients.x_dot_u, ...
                                                                    sampling_period, ...
                                                                    prediction_horizon, control_horizon, ...
                                                                    r, q_sample);
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
      lmi_norm_without_uncertainty_vec(ss_num, loop_index) = lmi_norm_without_uncertainty;

      closed_loop_ss = ss(Almi, Blmi, Clmi, Dlmi, -1);
      matlab_norm_without_uncertainty = norm(closed_loop_ss, inf);
      matlab_norm_without_uncertainty_vec(ss_num, loop_index) = matlab_norm_without_uncertainty;

      lmi_nmax_with_uncertainty_vec(ss_num, loop_index) = get_maximum_delay(lmi_norm_with_uncertainty);
      matlab_nmax_without_uncertainty_vec(ss_num, loop_index) = get_maximum_delay(matlab_norm_without_uncertainty);

      loop_index = loop_index + 1;
  end

  % =============================================================================
  if ss_num == 1
    figure1 = figure("Name", "Norms by LMI");
  else
    figure(figure1.Number)
  end
  % =============================================================================
  subplot(2,2,ss_num)
  plot_overlapping_norms(q, 90, lmi_norm_with_uncertainty_vec(ss_num, :), ...
                        matlab_norm_without_uncertainty_vec(ss_num, :), ...
                        {'With uncertainty', 'Without uncertainty'})
  % =============================================================================
  if ss_num == 1
    figure2 = figure("Name", "Maximum delay allowed by LMI and Matlab");
  else
    figure(figure2.Number)
  end
  % =============================================================================
  subplot(2,2,ss_num)
  plot_overlapping_nmax(q, 90, matlab_nmax_without_uncertainty_vec(ss_num, :), ...
                        lmi_nmax_with_uncertainty_vec(ss_num, :), ...
                        "", {'Without uncertainty', 'With uncertainty'})
  % =============================================================================

  close(waitbar_fig)
end

function max_delay = get_maximum_delay(norm)
  max_delay = 0;
  if norm < 1
    max_delay = floor(1/norm);
  end
end
