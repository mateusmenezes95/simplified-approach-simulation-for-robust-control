function plot_3d_path(desired, actual)
	desired_x = desired.pose(:, 1);
	desired_y = desired.pose(:, 2);
	desired_z = desired.pose(:, 3);
	plot3(desired_x, desired_y, desired_z, desired.line_spec, 'LineWidth', desired.line_width)
	grid on

	[x_min, x_max] = get_axis_limits(desired_x, 0.1);
	[y_min, y_max] = get_axis_limits(desired_y, 0.1);
	[z_min, z_max] = get_axis_limits(desired_z, 0.1);
	z_min = 0;

	axis([x_min, x_max, y_min, y_max, z_min, z_max])
	set(gca, 'ZDir', 'reverse')

	xlabel('x [m]')
	ylabel('y [m]')
	zlabel('z [m]')

	if nargin > 1
		actual_x = actual.pose(:, 1);
		actual_y = actual.pose(:, 2);
		actual_z = actual.pose(:, 3);
		hold on
		plot3(actual_x, actual_y, actual_z, actual.line_spec, 'LineWidth', actual.line_width)
		hold off

		legend('Desired', 'Actual')

		[actual_x_min, actual_x_max] = get_axis_limits(actual_x, 0.1);
		[actual_y_min, actual_y_max] = get_axis_limits(actual_y, 0.1);
		[actual_z_min, actual_z_max] = get_axis_limits(actual_z, 0.1);

		x_min = min(x_min, actual_x_min);
		x_max = max(x_max, actual_x_max);
		y_min = min(y_min, actual_y_min);
		y_max = max(y_max, actual_y_max);
		z_max = max(z_max, actual_z_max);

		axis([x_min, x_max, y_min, y_max, z_min, z_max])
	end
end
