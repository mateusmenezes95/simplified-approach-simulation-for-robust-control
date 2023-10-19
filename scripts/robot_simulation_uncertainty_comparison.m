current_script_path = fileparts(mfilename('fullpath'));

cd(current_script_path)

addpath(genpath("../lib"))
addpath(genpath("../lib/mpc_functions"))
addpath(genpath("../lib/chart_functions"))

run simulation_parameters
run robot_model

robot_vel = 0.05;
square_trajectory = generate_square_trajectory(1, robot_vel, sampling_period);
x_trajectory = square_trajectory(1,:);
y_trajectory = square_trajectory(2,:);
waypoints_qty = length(square_trajectory);

sim_time = 100;  % Choosen due results inpection

%==============================================================================
% Plot Parameters
%==============================================================================
save_figures = true;
font_size = 10;
line_thickness = 0.8;
y_axis_limits_offset = 0.2;
figure_idx = 1;
plot_step_response = false;
trajectory_limits =  [-0.05 1.05];
clipped_trajectory_start_time = 55;
clipped_trajectory_end_time = 65;
%==============================================================================
% Dynamic Model
%==============================================================================
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
%==============================================================================
% Dynamic Model With Uncertainties
%==============================================================================
robot_param = nominal_model;

mass_delta = 0.01;
inertia_delta = 0.10;
robot_radius_delta = 0.05;
armature_resistance_delta = 0.5;

uncertainty_mass_vec = [
  nominal_model.mass, ...
  nominal_model.mass*(1 - mass_delta), ...
  nominal_model.mass*(1 + mass_delta)
];

uncertainty_inertia_vec = [
  nominal_model.inertia, ...
  nominal_model.inertia*(1 - inertia_delta), ...
  nominal_model.inertia*(1 + inertia_delta)
];

uncertainty_robot_radius_vec = [
  nominal_model.robot_radius, ...
  nominal_model.robot_radius*(1 - robot_radius_delta), ...
  nominal_model.robot_radius*(1 + robot_radius_delta)
];

uncertainty_armature_resistance_vec = [
  nominal_model.armature_resistance, ...
  nominal_model.armature_resistance*(1 - armature_resistance_delta), ...
  nominal_model.armature_resistance*(1 + armature_resistance_delta)
];

r = 10000;
q = 550;
N = 2;

param_name = 'worst-scenario';
loop_step_params_str = ['q = ' num2str(q) ', r = ' num2str(r) ' and ' param_name];
print_section_description(['Running Robot Simulation on Simulink with parameters: ' loop_step_params_str])

plot_line_styles = ["-r", "-.b"];
clipped_plot_line_styles = plot_line_styles; 
figures = [];
control_signals_norm = [0 0];
control_signals_diff_norm = [0 0];

for i=1:2
    figure_idx = 1;

    if (i == 1)
      robot_param.mass = uncertainty_mass_vec(parameter_condition_as_int('nominal'));
      robot_param.inertia = uncertainty_inertia_vec(parameter_condition_as_int('nominal'));
      robot_param.robot_radius = uncertainty_robot_radius_vec(parameter_condition_as_int('nominal'));
      robot_param.armature_resistance = uncertainty_armature_resistance_vec(parameter_condition_as_int('nominal'));
      legend_str = 'Nominal';
    else
      robot_param.mass = uncertainty_mass_vec(parameter_condition_as_int('upper'));
      robot_param.inertia = uncertainty_inertia_vec(parameter_condition_as_int('lower'));
      robot_param.robot_radius = uncertainty_robot_radius_vec(parameter_condition_as_int('upper'));
      robot_param.armature_resistance = uncertainty_armature_resistance_vec(parameter_condition_as_int('lower'));
      legend_str = 'Polyhedron Vertex';
    end

    [Aaug, Baug, Caug, A, B, C, D] = get_model_matrices(robot_param, sampling_period);
    [Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
    [Kw, Kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);
    %==============================================================================

    %==============================================================================
    % Run Simulation
    %==============================================================================

    print_section_description(['Running Robot Simulation on Simulink to ' legend_str])
    sim_out = sim('../simulink/robot.slx');
    print_section_description("Robot Simulation Finished!")
    %==============================================================================

    %==============================================================================
    % Plot trajectories
    %==============================================================================
    [figures, figure_idx] = select_figure(figures, ['Trajectory | Condition: ' legend_str], i, figure_idx);
    if i == 1
        plot_robot_trajectory(x_trajectory, y_trajectory, 'Reference', '--k', line_thickness)
        xlim(trajectory_limits);
        ylim(trajectory_limits);
        hold on
    end
    plot_robot_trajectory(sim_out.x, sim_out.y, legend_str, plot_line_styles(i), line_thickness)
    hold on
    %==============================================================================

    %==============================================================================
    % Plot clipped trajectories
    %==============================================================================
    [figures, figure_idx] = select_figure(figures, ['Clipped Trajectory | Condition: ' legend_str], i, figure_idx);
    f = figure(figures(figure_idx-1));
    f.Position = [1435 712 732 617];
    if i == 1
        start_idx = find(sim_out.sim_time_sampled >= clipped_trajectory_start_time, 1);
        end_idx = find(sim_out.sim_time_sampled >= clipped_trajectory_end_time, 1);
        % plot_robot_trajectory(x_trajectory(start_idx:end_idx), y_trajectory(start_idx:end_idx), 'referencia', '--k', line_thickness)
        % hold on
    end
    plot_robot_trajectory(sim_out.x(start_idx:end_idx), sim_out.y(start_idx:end_idx),...
                          legend_str, clipped_plot_line_styles(i), line_thickness)
    legend({}, Location="southeast", FontSize=12)
    hold on
    %==============================================================================

    %==============================================================================
    % Plot control signals
    %==============================================================================
    [figures, figure_idx] = select_figure(figures, ['Control Signals | Condition: ' legend_str], i, figure_idx);
    plot_control_signals(sim_out.sim_time_sampled, [sim_out.u1 sim_out.u2 sim_out.u3], ...
                         legend_str, plot_line_styles(i), line_thickness, 'u_')
    hold on
    %==============================================================================

    %==============================================================================
    % Plot control signals increment
    %==============================================================================
    [figures, figure_idx] = select_figure(figures, ['Control Signals Increment | Condition: ' legend_str], i, figure_idx);
    plot_control_signals(sim_out.sim_time_sampled, [sim_out.delta_u1 sim_out.delta_u2 sim_out.delta_u3], ...
                         legend_str, plot_line_styles(i), line_thickness, '\Deltau_')
    hold on
    %==============================================================================

    %==============================================================================
    % Plot cliped control signals
    %==============================================================================
    [figures, figure_idx] = select_figure(figures, ['Clipped Control Signals | Condition: ' legend_str], i, figure_idx);
    f = figure(figures(figure_idx-1));
    f.Position = [1435 712 732 617];
    control_signals = [sim_out.u1(start_idx:end_idx) sim_out.u2(start_idx:end_idx) sim_out.u3(start_idx:end_idx)];
    plot_control_signals(sim_out.sim_time_sampled(start_idx:end_idx), control_signals, ...
                         legend_str, clipped_plot_line_styles(i), 1.5*line_thickness, 'u_')
    for m=1:3
        subplot(3,1,m)
        xlim([clipped_trajectory_start_time clipped_trajectory_end_time])
        legend(Location="northwest")
    end
    hold on
    %==============================================================================

    %==============================================================================
    % Plot cliped control signals increment
    %==============================================================================
    [figures, figure_idx] = select_figure(figures, ['Clipped Control Signals Increment | Condition: ' legend_str], i, figure_idx);
    f = figure(figures(figure_idx-1));
    f.Position = [1435 712 732 617];
    control_signals = [sim_out.delta_u1(start_idx:end_idx) sim_out.delta_u2(start_idx:end_idx) sim_out.delta_u3(start_idx:end_idx)];
    plot_control_signals(sim_out.sim_time_sampled(start_idx:end_idx), control_signals, ...
                         legend_str, clipped_plot_line_styles(i), 1.5*line_thickness, '\Deltau_')
    for m=1:3
        subplot(3,1,m)
        xlim([clipped_trajectory_start_time clipped_trajectory_end_time])
        legend(Location="northwest")
    end
    hold on
    %==============================================================================

    u_norm = sqrt(norm(sim_out.u1,2)^2 + norm(sim_out.u2,2)^2 + norm(sim_out.u3,2)^2); 
    du_norm = sqrt(norm(sim_out.delta_u1,2)^2 + norm(sim_out.delta_u2,2)^2 + norm(sim_out.delta_u3,2)^2); 
    control_signals_norm(i) = u_norm;
    control_signals_diff_norm(i) = du_norm;
end

figure()
subplot(1,3,1)
x_labels = {'Nominal', 'Polyhedron Vertex'};
b = bar(1:2, control_signals_norm);
xticklabels(x_labels);
y_tips = b.YEndPoints;
labels = string(b.YData);
text(b.XEndPoints,y_tips,labels,'HorizontalAlignment','center',...
'VerticalAlignment','bottom')
xlabel('Dynamic model parameters condition')
ylabel('||u||_2')

subplot(1,3,2)
b = bar(1:2, control_signals_diff_norm);
xticklabels(x_labels);
y_tips = b.YEndPoints;
labels = string(b.YData);
text(b.XEndPoints,y_tips,labels,'HorizontalAlignment','center',...
'VerticalAlignment','bottom')
xlabel('Dynamic model parameters condition')
ylabel('||\Deltau||_2')

subplot(1,3,3)
b = bar(1:2, control_signals_norm./control_signals_diff_norm);
xticklabels(x_labels);
y_tips = b.YEndPoints;
labels = string(b.YData);
text(b.XEndPoints,y_tips,labels,'HorizontalAlignment','center',...
'VerticalAlignment','bottom')
xlabel('Dynamic model parameters condition')
ylabel('||u||_2\\||\Deltau||_2')

figure_folder_name = 'axebot-simulation-path-following-comparison';

if save_figures
  lar_folder_path = fullfile(getenv('HOME'), 'Documents', 'lars', figure_folder_name);
  figures_folder = lar_folder_path;

  figures_names = ["trajectory",...
                  "clipped_trajectory",...
                  "control_signals",...
                  "control_signals_increment",...
                  "clipped_control_signals",...
                  "clipped_control_signals_increment",...
                  "control_signals_norm"];

  if ~isfolder(figures_folder)
    mkdir(figures_folder);
    disp('Folder created successfully.');
  end

  for i=1:length(figures)
      figure(i)
      saveas(i, fullfile(figures_folder, [figures_names(i) + ".eps"]), 'epsc')
  end

  disp("Figures saved successfully in " + figures_folder);
  close all
end

function i = parameter_condition_as_int(condition_str)
  switch condition_str
    case 'nominal'
      i = 1;
    case 'lower'
      i = 2;
    case 'upper'
      i = 3;
  end
end
