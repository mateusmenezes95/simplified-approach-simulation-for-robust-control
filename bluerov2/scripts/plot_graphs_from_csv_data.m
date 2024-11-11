clc
clear

without_delay = "gazebo-simulation-without-delay.csv";
with_2_samples_delay = "gazebo-simulation-with-max-delay-equal-to-2.csv";
with_4_samples_delay = "gazebo-simulation-with-max-delay-equal-to-14.csv";

selection = 2; % 1 - without_delay, 2 - with_2_samples_delay, 3 - with_4_samples_delay

if selection == 1
    csv_Selected = without_delay;
elseif selection == 2
    csv_Selected = with_2_samples_delay;
else
    csv_Selected = with_4_samples_delay;
end

current_script_path = fileparts(mfilename('fullpath'));
csv_file_path = fullfile(current_script_path, '../datalogs', 'gazebo-sim_10-11-2024', csv_Selected);

%===================================================================================================
% Plot Parameters
%===================================================================================================
font_size = 10;
line_thickness = 1.5;
y_axis_limits_offset = 0.2;
figure_idx = 1;

current_script_path = fileparts(mfilename('fullpath'));
cd(current_script_path)

% Add paths so that we can use the functions from the files in the lib folder
addpath(genpath("."))
addpath(genpath("../functions/plots"))

[ctrl_inputs.t, ctrl_inputs.values] = read_and_process_data(csv_file_path, [
    "x__time", ...
    "x_unconstrained_mpc_mpc_data_ctrl_input_force_x", ...
    "x_unconstrained_mpc_mpc_data_ctrl_input_force_y", ...
    "x_unconstrained_mpc_mpc_data_ctrl_input_force_z", ...
    "x_unconstrained_mpc_mpc_data_ctrl_input_torque_z"
]);

[robot_traj.t, robot_traj.values] = read_and_process_data(csv_file_path, [
    "x__time", ...
    "x_unconstrained_mpc_mpc_data_robot_vel_linear_x", ...
    "x_unconstrained_mpc_mpc_data_robot_vel_linear_y", ...
    "x_unconstrained_mpc_mpc_data_robot_vel_linear_z", ...
    "x_unconstrained_mpc_mpc_data_robot_vel_angular_z"
]);

[desired_traj.t, desired_traj.values] = read_and_process_data(csv_file_path, [
    "x__time", ...
    "x_unconstrained_mpc_mpc_data_desired_robot_vel_linear_x", ...
    "x_unconstrained_mpc_mpc_data_desired_robot_vel_linear_y", ...
    "x_unconstrained_mpc_mpc_data_desired_robot_vel_linear_z", ...
    "x_unconstrained_mpc_mpc_data_desired_robot_vel_angular_z"
]);

[delay.t, delay.values] = read_and_process_data(csv_file_path, [
    "x__time", ...
    "x_unconstrained_mpc_mpc_data_delay_data"
]);

figure("Name", "velocities-in-body-fixed-frame")
desired_body_fixed_vel_args.y_labels = {'u [m/s]', 'v [m/s]', 'w [m/s]', 'r [rad/s]'};
desired_body_fixed_vel_args.y_min_offset = 0.1;
desired_body_fixed_vel_args.y_max_offset = 0.1;
plot_per_dof_values(desired_traj.t, desired_body_fixed_vel_args, desired_traj.values, robot_traj.values);

error = abs(desired_traj.values - robot_traj.values);
trapz(desired_traj.t, error)

figure("Name", "velocities-error-in-body-fixed-frame")
error_body_fixed_vel_args.y_labels = {'u [m/s]', 'v [m/s]', 'w [m/s]', 'r [rad/s]'};
error_body_fixed_vel_args.y_min_offset = 0.005;
error_body_fixed_vel_args.y_max_offset = 0.005;
plot_per_dof_values(desired_traj.t, error_body_fixed_vel_args, error)

figure("Name", "bluerov-control-signals")
plot_generalized_forces(ctrl_inputs.t, ctrl_inputs.values', -1, '-r', line_thickness)

figure("Name", "delay")

% Filter the data to include only values between 10 and 20 seconds
t_start = 10;
t_end = 20;
time_range = delay.t >= t_start & delay.t <= t_end;
filtered_time = delay.t(time_range);
filtered_values = delay.values(time_range);

plot(filtered_time, filtered_values)
xlim([t_start t_end])
if selection == 2
    ylim([0.0 max(delay.values)])
else
    ylim([0.0 max(delay.values) + 0.1])
end

ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3) - 0.01;
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

function [t, data] = read_and_process_data(csv_file_path, selected_vars)
    % READ_AND_PROCESS_CTRL_INPUTS Reads and processes control inputs from a CSV file
    %
    %   data = READ_AND_PROCESS_CTRL_INPUTS(csv_file_path, selected_vars)
    %   reads control inputs from the specified CSV file, processes the time
    %   data, and removes missing values.
    %
    %   Inputs:
    %       csv_file_path - Path to the CSV file containing the data
    %       selected_vars - Cell array of selected variable names to read from the CSV file
    %Z
    %   Output:
    %       data - Table containing the processed control inputs
    %
    %   Example:
    %       selected_vars = {
    %           "x__time", ...
    %           "x_unconstrained_mpc_control_input_force_x", ...
    %           "x_unconstrained_mpc_control_input_force_y", ...
    %           "x_unconstrained_mpc_control_input_force_z", ...
    %           "x_unconstrained_mpc_control_input_torque_z"
    %       };
    %       data = read_and_process_data('/path/to/csv-data-test.csv', selected_vars);

    % Read control inputs
    ctrl_input_opts = detectImportOptions(csv_file_path);
    ctrl_input_opts.SelectedVariableNames = selected_vars;
    data = readtable(csv_file_path, ctrl_input_opts);
    % data = rmmissing(data);

    % Process time data for control inputs
    old_values = data.x__time;
    data.x__time(1) = 0;
    for i = 2:size(data, 1)
        data.x__time(i) = old_values(i) - old_values(i - 1) + data.x__time(i - 1);
    end
    data = table2array(data);
    t = data(:, 1);
    data = data(:, 2:end);
end
