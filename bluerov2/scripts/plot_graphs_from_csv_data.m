clc
clear

csv_file_path = '/home/mateus/ufba_ws/automatica-paper-scripts/bluerov2/datalogs/csv-data-test.csv';

ctrl_inputs = read_and_process_data(csv_file_path, [
    "x__time", ...
    "x_unconstrained_mpc_control_input_force_x", ...
    "x_unconstrained_mpc_control_input_force_y", ...
    "x_unconstrained_mpc_control_input_force_z", ...
    "x_unconstrained_mpc_control_input_torque_z"
]);

robot_traj = read_and_process_data(csv_file_path, [
    "x__time", ...
    "x_unconstrained_mpc_robot_vel_linear_x", ...
    "x_unconstrained_mpc_robot_vel_linear_y", ...
    "x_unconstrained_mpc_robot_vel_linear_z", ...
    "x_unconstrained_mpc_robot_vel_angular_z"
]);

desired_traj = read_and_process_data(csv_file_path, [
    "x__time", ...
    "x_unconstrained_mpc_desired_robot_vel_linear_x", ...
    "x_unconstrained_mpc_desired_robot_vel_linear_y", ...
    "x_unconstrained_mpc_desired_robot_vel_linear_z", ...
    "x_unconstrained_mpc_desired_robot_vel_angular_z"
]);

function data = read_and_process_data(csv_file_path, selected_vars)
    % READ_AND_PROCESS_CTRL_INPUTS Reads and processes control inputs from a CSV file
    %
    %   data = READ_AND_PROCESS_CTRL_INPUTS(csv_file_path, selected_vars)
    %   reads control inputs from the specified CSV file, processes the time
    %   data, and removes missing values.
    %
    %   Inputs:
    %       csv_file_path - Path to the CSV file containing the data
    %       selected_vars - Cell array of selected variable names to read from the CSV file
    %
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
    data = rmmissing(data);

    % Process time data for control inputs
    old_values = data.x__time;
    data.x__time(1) = 0;
    for i = 2:size(data, 1)
        data.x__time(i) = old_values(i) - old_values(i - 1) + data.x__time(i - 1);
    end
end

function plot_per_dof_values(t, args, desired_values, actual_values)
	dofs = size(desired_values, 2);
	for i = 1:dofs
		subplot(dofs, 1, i)
		plot(t, desired_values(:, i), 'r', 'LineWidth', 1.5)
		ylabel(args.y_labels{i}, 'Interpreter', 'latex')

		y_min = min(desired_values(:, i)) - args.y_min_offset;
		y_max = max(desired_values(:, i)) + args.y_max_offset;
		
		xlim([min(t) max(t)])
		ylim([y_min y_max])
		grid on
	end
	xlabel('Time [s]')

	if nargin > 3
		for i = 1:dofs
			subplot(dofs, 1, i)
			hold on
			plot(t, actual_values(:, i), 'b', 'LineWidth', 1.5)
			hold off
		end
		legend('Desejado', 'Atual')
	end
end
